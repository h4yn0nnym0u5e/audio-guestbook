/**
 * Audio Guestbook, Copyright (c) 2022 Playful Technology
 * 
 * Tested using a Teensy 4.0 with Teensy Audio Shield, although should work 
 * with minor modifications on other similar hardware
 * 
 * When handset is lifted, a pre-recorded greeting message is played, followed by a tone.
 * Then, recording starts, and continues until the handset is replaced.
 * Playback button allows all messages currently saved on SD card through earpiece 
 * 
 * Files are saved on SD card as 44.1kHz, 16-bit, mono signed integer RAW audio format 
 * --> changed this to WAV recording, DD4WH 2022_07_31
 * --> added MTP support, which enables copying WAV files from the SD card via the USB connection, DD4WH 2022_08_01
 * 
 * 
 * Frank DD4WH, August 1st 2022 
 * for a DBP 611 telephone (closed contact when handheld is lifted) & with recording to WAV file
 * contact for switch button 0 is closed when handheld is lifted
 * 
 * GNU GPL v3.0 license
 * 
 */

#include <Bounce.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TimeLib.h>
#include <MTP_Teensy.h>
#include "play_sd_wav.h" // local copy with fixes
#include "phone8746.h"

// DEFINES
// Define pins used by Teensy Audio Shield
//#define SDCARD_CS_PIN    10
#define SDCARD_CS_PIN    BUILTIN_SDCARD
#define SDCARD_MOSI_PIN  7
#define SDCARD_SCK_PIN   14

// And those used for inputs
#define HOOK_PIN 25
#define PLAYBACK_BUTTON_PIN 1
#define SW1_PIN 40 // goes low during dialling: used to re-start SW3 count
#define SW3_PIN 26 // one high pulse per count during dialling

#define MAX_RECORDING_TIME_MS 120000 // prevent failed hang-ups filling SD card

#define noINSTRUMENT_SD_WRITE
#define SGTL_ADDRESS LOW

// GLOBALS
// Audio initialisation code can be generated using the GUI interface at https://www.pjrc.com/teensy/gui/
// Inputs
AudioSynthWaveform          waveform1; // To create the "beep" sfx
AudioInputI2S               i2s2; // I2S input from microphone on audio shield
AudioPlaySdWavX              playGreeting; // Play 44.1kHz 16-bit PCM greeting WAV file
AudioPlaySdWavX              playMessage;  // Play 44.1kHz 16-bit PCM message WAV file
AudioRecordQueue            queue1; // Creating an audio buffer in memory before saving to SD
AudioMixer4                 mixer; // Allows merging several inputs to same output
AudioOutputI2S              i2s1; // I2S interface to Speaker/Line Out on Audio shield
AudioConnection patchCord1(waveform1, 0, mixer, 0); // wave to mixer 
AudioConnection patchCord2(playGreeting, 0, mixer, 1); // greeting file playback mixer
AudioConnection patchCord3(playMessage, 0, mixer, 2); // message file playback mixer
AudioConnection patchCord4(mixer, 0, i2s1, 0); // mixer output to speaker (L)
AudioConnection patchCord6(mixer, 0, i2s1, 1); // mixer output to speaker (R)
AudioConnection patchCord5(i2s2, 0, queue1, 0); // mic input to queue (L)
AudioControlSGTL5000      sgtl5000_1;

// Filename to save audio recording on SD card
char filename[15];
// The file object itself
File frec;
int nextRecNumber;

// Use long 40ms debounce time on both switches
Bounce buttonRecord = Bounce(HOOK_PIN, 40);
Bounce buttonPlay = Bounce(PLAYBACK_BUTTON_PIN, 40);
#define HOOK_ACTIVATED buttonRecord.fallingEdge
#define HOOK_DEACTIVATED buttonRecord.risingEdge
#define PLAY_ACTIVATED buttonPlay.fallingEdge
#define PLAY_DEACTIVATED buttonPlay.risingEdge

// Use pulse dialling to play back selected message
PulseDial dial(SW1_PIN,SW3_PIN);
#define MESSAGE_START_DELAY_MS 2000 // pause to allow another digit to be dialled
int theNumber; // the number you have dialled is...

// Keep track of current state of the device
elapsedMillis theTimer;
enum Mode {Initialising, Ready, WaitPrompt, Prompting, PromptBeep, Recording, EndBeeps, Playing, Dialling};
Mode mode = Mode::Initialising;

float beep_volume = 0.4f; // not too loud :-)
uint32_t beepState;
#define END_BEEP_TIME 250        

uint32_t MTPcheckInterval; // default value of device check interval [ms]

// variables for writing to WAV file
unsigned long ChunkSize = 0L;
unsigned long Subchunk1Size = 16;
unsigned int AudioFormat = 1;
unsigned int numChannels = 1;
unsigned long sampleRate = 44100;
unsigned int bitsPerSample = 16;
unsigned long byteRate = sampleRate*numChannels*(bitsPerSample/8);// samplerate x channels x (bitspersample / 8)
unsigned int blockAlign = numChannels*bitsPerSample/8;
unsigned long Subchunk2Size = 0L;
unsigned long recByteSaved = 0L;
unsigned long NumSamples = 0L;
byte byte1, byte2, byte3, byte4;


void setNextRecNumber(void)
{
  nextRecNumber = findFileNumber(0,1);
  Serial.printf("Next recording will be %d in %s\n",nextRecNumber,filename);  
}


void setup() {

  Serial.begin(9600);
  while (!Serial && millis() < 5000) {
    // wait for serial port to connect.
  }
  Serial.println("Serial set up correctly");
  if (CrashReport)
    Serial.print(CrashReport);
  Serial.printf("Audio block set to %d samples\n",AUDIO_BLOCK_SAMPLES);
  print_mode();
  // Configure the input pins
  pinMode(HOOK_PIN, INPUT_PULLUP);
  pinMode(PLAYBACK_BUTTON_PIN, INPUT_PULLUP);

  // Audio connections require memory, and the record queue
  // uses this memory to buffer incoming audio.
  // 60 blocks of 256 samples is enough buffer for 348ms
  AudioMemory(60);

  // Enable the audio shield, select input, and enable output
  sgtl5000_1.setAddress(SGTL_ADDRESS);
  sgtl5000_1.enable();
  // Define which input on the audio shield to use (AUDIO_INPUT_LINEIN / AUDIO_INPUT_MIC)
  sgtl5000_1.inputSelect(AUDIO_INPUT_MIC);
  //sgtl5000_1.adcHighPassFilterDisable(); //

  // ----- Level settings -----
  sgtl5000_1.micGain(24); // set to suit your microphone
  sgtl5000_1.volume(0.5); // overall speaker volume

  mixer.gain(0, 0.1f); // beeps
  mixer.gain(1, 0.1f); // greeting
  mixer.gain(2, 1.0f); // message playback
  // --------------------------

  // Initialize the SD card
  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  if (!(SD.begin(SDCARD_CS_PIN))) 
  {
    // stop here if no SD card, but print a message
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }
    else Serial.println("SD card correctly initialized");

  setNextRecNumber();

  // mandatory to begin the MTP session.
    MTP.begin();

  // Add SD Card
//    MTP.addFilesystem(SD, "SD Card");
    MTP.addFilesystem(SD, "Kais Audio guestbook"); // choose a nice name for the SD card volume to appear in your file explorer
    Serial.println("Added SD card via MTP");
    MTPcheckInterval = MTP.storage()->get_DeltaDeviceCheckTimeMS();
    
    // Value in dB
//  sgtl5000_1.micGain(15);
//  sgtl5000_1.micGain(5); // much lower gain is required for the AOM5024 electret capsule

  // Synchronise the Time object used in the program code with the RTC time provider.
  // See https://github.com/PaulStoffregen/Time
  setSyncProvider(getTeensy3Time);
  
  // Define a callback that will assign the correct datetime for any file system operations
  // (i.e. saving a new audio recording onto the SD card)
  FsDateTime::setCallback(dateTime);

  MTP.loop();
  mode = Mode::Ready; print_mode();
  
  // Play a beep to indicate system is online
  waveform1.begin(beep_volume, 440, WAVEFORM_SINE);
  delay(1000);
  waveform1.amplitude(0);
}

void loop() {
  // First, read the buttons
  buttonRecord.update();
  buttonPlay.update();
  dial.update();

  switch(mode){
    case Mode::Ready:
      // An edge occurs when the handset is lifted
      if (HOOK_ACTIVATED()) 
      {
        Serial.println("Handset lifted");
        mode = Mode::WaitPrompt; print_mode();
        theTimer = 0;
      }
      else if (PLAY_ACTIVATED()) 
      {
        playLastRecording();
      }
      else if (dial.dialling())
      {
        mode = Mode::Dialling; print_mode();
        theNumber = 0;
      }
      break;

    case Mode::WaitPrompt:
      // Wait a second for users to put the handset to their ear
      if (theTimer >= 1000)
      {
        // Debug message
        Serial.println("Starting greeting");
        setMTPdeviceChecks(false); // disable MTP device checks while playing
        // Play the greeting inviting them to record their message
        playGreeting.play("greeting.wav"); 
        delay(5);   
        mode = Mode::Prompting; print_mode();
      }
      
      // Handset is replaced
      if (HOOK_DEACTIVATED())
      {  
        setMTPdeviceChecks(true); // re-enable MTP device checks
        mode = Mode::Ready; print_mode(); 
      }
      if (PLAY_ACTIVATED()) 
      {
        setMTPdeviceChecks(true); // re-enable MTP device checks
        playLastRecording();
      }
      break;
      
    case Mode::Prompting:
      // Wait until the  message has finished playing
      if (playGreeting.isStopped()) 
      {
        // Debug message
        Serial.println("Starting beep");
        setMTPdeviceChecks(true); // re-enable MTP device checks
        // Play the tone sound effect
        waveform1.begin(beep_volume, 440, WAVEFORM_SINE);
        theTimer = 0;
        mode = Mode::PromptBeep; print_mode();
      }
      
      // Handset is replaced
      if (HOOK_DEACTIVATED())
      { 
        playGreeting.stop();
        mode = Mode::Ready; print_mode();
      }
      if (PLAY_ACTIVATED()) 
      {
        playGreeting.stop();
        playLastRecording();
      }      
      break;

    case PromptBeep:
      if (theTimer >= 1250)
      {
        // Debug message
        Serial.println("Starting Recording");
        waveform1.amplitude(0);
        // Start the recording function
        startRecording();
        theTimer = 0; // prepare to time out if handset left off-hook
      }
      
      // Handset is replaced
      if (HOOK_DEACTIVATED())
      { 
        waveform1.amplitude(0);
        mode = Mode::Ready; print_mode();
      }
      if (PLAY_ACTIVATED()) 
      {
        waveform1.amplitude(0);
        playLastRecording();
      }      
      break;      

    case Mode::Recording:
      // Handset is replaced...
      if (HOOK_DEACTIVATED()
       || theTimer >= MAX_RECORDING_TIME_MS) // ...or has been off-hook too long
      {
        // Debug log
        Serial.println("Stopping Recording");
        // Stop recording
        stopRecording();
        setNextRecNumber(); // so dialling works as expected
        // Play audio tone to confirm recording has ended
        end_Beep(0b010101010);
      }
      else 
      {
        continueRecording();
      }
      break;

    case EndBeeps:
      if (theTimer > END_BEEP_TIME)
      {
        //Serial.println("beep");
        if (0 != (beepState & 2)) // do a beep
        {
          waveform1.frequency(523.25);
          waveform1.amplitude(beep_volume);
        }
        else
          waveform1.amplitude(0);

        beepState >>= 1;
        if (0 != beepState)
          theTimer = 0;            
        else
        {
          mode = Mode::Ready; 
          print_mode();
        }
      }
      break;   

    // Dial back by N messages, so dial 1 to review your
    // just-left message. Should we do something humorous 
    // for 999 or 911 or 112?! Or serious, even...
    case Mode::Dialling:
      if (dial.newNumber) // new set of pulses has arrived
      {
        theNumber = theNumber*10 + dial.count;
        dial.newNumber = false;
        Serial.printf("Number is now %d\n",theNumber);
      }

      if (dial.dialling()) // still dialling?
        theTimer = 0;      // reset the timeout
      else
      {
        if (theTimer > MESSAGE_START_DELAY_MS)
          playLastRecording(theNumber); // go back this many recordings
      }

      // Handset is replaced
      if (HOOK_DEACTIVATED())
      { 
        mode = Mode::Ready; print_mode();
      }
      break;

    case Mode::Playing: // to make compiler happy
      break;  

    case Mode::Initialising: // to make compiler happy
      break;  
  }   
  
  MTP.loop();  // This is mandatory to be placed in the loop code.
}

void setMTPdeviceChecks(bool nable)
{
  if (nable)
  {
    MTP.storage()->set_DeltaDeviceCheckTimeMS(MTPcheckInterval);
    Serial.print("En");
  }
  else
  {
    MTP.storage()->set_DeltaDeviceCheckTimeMS((uint32_t) -1);
    Serial.print("Dis");
  }
  Serial.println("abled MTP storage device checks");
}
  

#if defined(INSTRUMENT_SD_WRITE)
static uint32_t worstSDwrite, printNext;
#endif // defined(INSTRUMENT_SD_WRITE)

void makeFileName(int i)
{
  snprintf(filename, 11, " %05d.wav", i);
}

// Search for a file by its number.
// If inc is positive we are searching for the next non-existent file,
// if negative we're searching backwards for the previous existing file
// if zero, just heck for the existence of that file number
// returns >=0 for the file number [not] found; filename is set
int findFileNumber(int n, int inc)
{
  int result = -1;

  while (result < 0)
  {
    makeFileName(n);
    if (SD.exists(filename))
    {
      if (inc <= 0)
        result = n;
    }
    else
    {
      if (inc > 0)
        result = n;      
    }

    if (0 != inc)
      n += inc;
    else
      break;      
  }

  return result;
}

void startRecording() {
  setMTPdeviceChecks(false); // disable MTP device checks while recording
#if defined(INSTRUMENT_SD_WRITE)
  worstSDwrite = 0;
  printNext = 0;
#endif // defined(INSTRUMENT_SD_WRITE)
  // Find the first available file number
  int recNum = findFileNumber(nextRecNumber,1);
  if (recNum > 0)
    nextRecNumber = recNum;
    
  frec = SD.open(filename, FILE_WRITE);
  Serial.println("Opened file !");
  if(frec) {
    Serial.print("Recording to ");
    Serial.println(filename);
    queue1.begin();
    mode = Mode::Recording; print_mode();
    recByteSaved = 0L;
    theTimer = 0;
  }
  else {
    Serial.println("Couldn't open file to record!");
    mode = Mode::Ready; print_mode();
  }
}

void continueRecording() {
#if defined(INSTRUMENT_SD_WRITE)
  uint32_t started = micros();
#endif // defined(INSTRUMENT_SD_WRITE)

// Assume the audio block size is a (sub)multiple of an SD card sector,
// and write out large chunks at once for improved speed.
// e.g. block size of 256, NBLOX=16, 2 bytes per sample
// writes 16 sectors (8kbytes) at a time, every 93ms or so
#define NBLOX 16  
  // Check if there is data in the queue
  if (queue1.available() >= NBLOX) {
    byte buffer[NBLOX*AUDIO_BLOCK_SAMPLES*sizeof(int16_t)];
    // Fetch 2 blocks from the audio library and copy
    // into a 512 byte buffer.  The Arduino SD library
    // is most efficient when full 512 byte sector size
    // writes are used.
    for (int i=0;i<NBLOX;i++)
    {
      memcpy(buffer+i*AUDIO_BLOCK_SAMPLES*sizeof(int16_t), queue1.readBuffer(), AUDIO_BLOCK_SAMPLES*sizeof(int16_t));
      queue1.freeBuffer();
    }
    // Write all 512 bytes to the SD card
    frec.write(buffer, sizeof buffer);
    recByteSaved += sizeof buffer;
  }
  
#if defined(INSTRUMENT_SD_WRITE)
  started = micros() - started;
  if (started > worstSDwrite)
    worstSDwrite = started;

  if (millis() >= printNext)
  {
    Serial.printf("Worst write took %luus\n",worstSDwrite);
    worstSDwrite = 0;
    printNext = millis()+250;
  }
#endif // defined(INSTRUMENT_SD_WRITE)
}

void stopRecording() {
  // Stop adding any new data to the queue
  queue1.end();
  // Flush all existing remaining data from the queue
  while (queue1.available() > 0) {
    // Save to open file
    frec.write((byte*)queue1.readBuffer(), AUDIO_BLOCK_SAMPLES*sizeof(int16_t));
    queue1.freeBuffer();
    recByteSaved += AUDIO_BLOCK_SAMPLES*sizeof(int16_t);
  }
  writeOutHeader();
  // Close the file
  frec.close();
  Serial.println("Closed file");
  mode = Mode::Ready; print_mode();
  setMTPdeviceChecks(true); // enable MTP device checks, recording is finished
}


void playAllRecordings() 
{
  bool hungUp = false;
  // Recording files are saved in the root directory
  File dir = SD.open("/");
  
  while (!hungUp) 
  {
    File entry =  dir.openNextFile();
    
    if (!entry) // no more files
      break;
    
    if (strstr(entry.name(), "greeting"))
      continue;
      
    if (strstr(entry.name(), "emergency"))
      continue;
      
    int pos = strlen(entry.name()) - 4;
    // check we have a wav:
    if (0 != stricmp(entry.name()+pos, ".wav")) 
      continue;
      
    Serial.print("Now playing ");
    Serial.println(entry.name());
    // Play a short beep before each message
    waveform1.amplitude(beep_volume);
    delay(750);
    waveform1.amplitude(0);
    // Play the file
    playMessage.play(entry.name());
    mode = Mode::Playing; print_mode();
    entry.close();

    while (!playMessage.isStopped())  // this works for playWav
    {
      buttonPlay.update();
      buttonRecord.update();
      // Early hang-up, or release and press play
      if (PLAY_ACTIVATED() || HOOK_DEACTIVATED()) 
      {
        playMessage.stop();
        mode = Mode::Ready; print_mode();
        hungUp = true; // exit enclosing while() loop
        break;
      } 
      yield(); // just in case    
    }
  }
  
  // All files have been played
  dir.close();
  end_Beep(0b101010);
}

void playLastRecording(void) { playLastRecording(1); }
void playLastRecording(int backBy) {
  // Find the first available file number
  int fileNumber = findFileNumber(nextRecNumber - backBy,-1); // search back to a real file

  if (fileNumber >= 0) // found something to play
  {
    // now play file 
    Serial.println(filename);
    playMessage.play(filename);
    mode = Mode::Playing; print_mode();
    while (!playMessage.isStopped())  // this works for playWav
    {
      buttonPlay.update();
      buttonRecord.update();
      // Early hang-up, or release and press play
      if (PLAY_ACTIVATED() || HOOK_DEACTIVATED()) 
      {
        playMessage.stop();
        mode = Mode::Ready; print_mode();
        break;
      } 
      yield(); // just in case  
    }
  }
  // file has been played
  end_Beep(0b0101010);
}


// Retrieve the current time from Teensy built-in RTC
time_t getTeensy3Time(){
  return Teensy3Clock.get();
}

// Callback to assign timestamps for file system operations
void dateTime(uint16_t* date, uint16_t* time, uint8_t* ms10) {

  // Return date using FS_DATE macro to format fields.
  *date = FS_DATE(year(), month(), day());

  // Return time using FS_TIME macro to format fields.
  *time = FS_TIME(hour(), minute(), second());

  // Return low time bits in units of 10 ms.
  *ms10 = second() & 1 ? 100 : 0;
}

// Non-blocking delay, which pauses execution of main program logic,
// but while still listening for input 
void waitx(unsigned int milliseconds) {
  elapsedMillis msec=0;

  while (msec <= milliseconds) {
    buttonRecord.update();
    buttonPlay.update();
    if (buttonRecord.fallingEdge()) Serial.println("Button (pin 0) Press");
    if (buttonPlay.fallingEdge()) Serial.println("Button (pin 1) Press");
    if (buttonRecord.risingEdge()) Serial.println("Button (pin 0) Release");
    if (buttonPlay.risingEdge()) Serial.println("Button (pin 1) Release");
  }
}


void writeOutHeader() { // update WAV header with final filesize/datasize

//  NumSamples = (recByteSaved*8)/bitsPerSample/numChannels;
//  Subchunk2Size = NumSamples*numChannels*bitsPerSample/8; // number of samples x number of channels x number of bytes per sample
  Subchunk2Size = recByteSaved - 42; // because we didn't make space for the header to start with! Lose 21 samples...
  ChunkSize = Subchunk2Size + 34; // was 36;
  frec.seek(0);
  frec.write("RIFF");
  byte1 = ChunkSize & 0xff;
  byte2 = (ChunkSize >> 8) & 0xff;
  byte3 = (ChunkSize >> 16) & 0xff;
  byte4 = (ChunkSize >> 24) & 0xff;  
  frec.write(byte1);  frec.write(byte2);  frec.write(byte3);  frec.write(byte4);
  frec.write("WAVE");
  frec.write("fmt ");
  byte1 = Subchunk1Size & 0xff;
  byte2 = (Subchunk1Size >> 8) & 0xff;
  byte3 = (Subchunk1Size >> 16) & 0xff;
  byte4 = (Subchunk1Size >> 24) & 0xff;  
  frec.write(byte1);  frec.write(byte2);  frec.write(byte3);  frec.write(byte4);
  byte1 = AudioFormat & 0xff;
  byte2 = (AudioFormat >> 8) & 0xff;
  frec.write(byte1);  frec.write(byte2); 
  byte1 = numChannels & 0xff;
  byte2 = (numChannels >> 8) & 0xff;
  frec.write(byte1);  frec.write(byte2); 
  byte1 = sampleRate & 0xff;
  byte2 = (sampleRate >> 8) & 0xff;
  byte3 = (sampleRate >> 16) & 0xff;
  byte4 = (sampleRate >> 24) & 0xff;  
  frec.write(byte1);  frec.write(byte2);  frec.write(byte3);  frec.write(byte4);
  byte1 = byteRate & 0xff;
  byte2 = (byteRate >> 8) & 0xff;
  byte3 = (byteRate >> 16) & 0xff;
  byte4 = (byteRate >> 24) & 0xff;  
  frec.write(byte1);  frec.write(byte2);  frec.write(byte3);  frec.write(byte4);
  byte1 = blockAlign & 0xff;
  byte2 = (blockAlign >> 8) & 0xff;
  frec.write(byte1);  frec.write(byte2); 
  byte1 = bitsPerSample & 0xff;
  byte2 = (bitsPerSample >> 8) & 0xff;
  frec.write(byte1);  frec.write(byte2); 
  frec.write("data");
  byte1 = Subchunk2Size & 0xff;
  byte2 = (Subchunk2Size >> 8) & 0xff;
  byte3 = (Subchunk2Size >> 16) & 0xff;
  byte4 = (Subchunk2Size >> 24) & 0xff;  
  frec.write(byte1);  frec.write(byte2);  frec.write(byte3);  frec.write(byte4);
  frec.close();
  Serial.println("header written"); 
  Serial.print("Subchunk2: "); 
  Serial.println(Subchunk2Size); 
}

void end_Beep(uint32_t pattern) 
{
    mode = Mode::EndBeeps; print_mode();
    theTimer = END_BEEP_TIME + 1;
    beepState = pattern;
}


//enum Mode {Initialising, Ready, WaitPrompt, Prompting, PromptBeep, Recording, EndBeeps, Playing, Dialling};
#define OR_PRINT_MODE(x) else if (mode == Mode::x) Serial.println(#x)
void print_mode(void) { // only for debugging
  static int lastPrint = -1;
  if (mode != lastPrint)
  {
    Serial.print("Mode switched to: ");
    // Initialising, Ready, Prompting, Recording, Playing ...
    if(mode == Mode::Ready) Serial.println(" Ready");
    OR_PRINT_MODE(WaitPrompt);
    OR_PRINT_MODE(Prompting);
    OR_PRINT_MODE(PromptBeep);
    OR_PRINT_MODE(Recording);
    OR_PRINT_MODE(EndBeeps);
    OR_PRINT_MODE(Playing);
    OR_PRINT_MODE(Dialling);
    OR_PRINT_MODE(Initialising);
    else Serial.println(" Undefined");
  }
  else
    Serial.print('.');
  lastPrint = mode;
}
