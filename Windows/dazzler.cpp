// -----------------------------------------------------------------------------
// Cromemco Dazzler emulation for Windows
// Copyright (C) 2018 David Hansel
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software Foundation,
// Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
// -----------------------------------------------------------------------------

#ifdef _WIN32

#include <winsock2.h>
#include <windows.h>
#include <windowsx.h>
#include <ws2tcpip.h>
#include <wingdi.h>
#include <setupapi.h>
#include <Mmdeviceapi.h>
#include <audioclient.h>
#include <avrt.h>
#include <sys/timeb.h>
#include <d2d1.h>
#include <d2d1helper.h>


#define DAZ_MEMBYTE   0x10
#define DAZ_FULLFRAME 0x20
#define DAZ_CTRL      0x30
#define DAZ_CTRLPIC   0x40
#define DAZ_DAC       0x50
#define DAZ_VERSION   0xF0

#define DAZ_JOY1      0x10
#define DAZ_JOY2      0x20
#define DAZ_KEY       0x30

// features
#define FEAT_VIDEO    0x01
#define FEAT_JOYSTICK 0x02
#define FEAT_DUAL_BUF 0x04
#define FEAT_VSYNC    0x08
#define FEAT_DAC      0x10
#define FEAT_KEYBOARD 0x20
#define FEAT_FRAMEBUF 0x40

// computer/dazzler version
#define DAZZLER_VERSION 0x02
int computer_version =  0x00;


COLORREF colors[16] = {RGB(0x00,0x00,0x00), RGB(0x80,0x00,0x00), RGB(0x00,0x80,0x00), RGB(0x80,0x80,0x00),
                       RGB(0x00,0x00,0x80), RGB(0x80,0x00,0x80), RGB(0x00,0x80,0x80), RGB(0x80,0x80,0x80),
                       RGB(0x00,0x00,0x00), RGB(0xff,0x00,0x00), RGB(0x00,0xff,0x00), RGB(0xff,0xff,0x00),
                       RGB(0x00,0x00,0xff), RGB(0xff,0x00,0xff), RGB(0x00,0xff,0xff), RGB(0xff,0xff,0xff)};

#define RGB2BGR(v) ((((v)&0xFF0000)>>16) | ((v) & 0x00FF00) | (((v) & 0x0000FF)<<16))


// D7: not used
// D6: 1=resolution x4, 0=normal resolution
// D5: 1=2k memory, 0=512byte memory
// D4: 1=color, 0=monochrome
// D3-D0: color info for x4 high res mode
byte dazzler_picture_ctrl = 0;

// D7: on/off
// D6-D0: screen memory location (not used in client)
byte dazzler_ctrl = 0;

// dazzler video memory, keeping two buffers
byte dazzler_mem[2*2048];

// joystick 
int g_joy_swap = 0, g_joy_show = 0, g_joy_keys = 0;
byte g_joy1[3], g_joy2[3];

wchar_t *joyKeyRegNames[12] = {L"Joy%iUp", L"Joy%iDown", L"Joy%iLeft", L"Joy%iRight",
                               L"Joy%iButton1", L"Joy%iButton2", L"Joy%iButton3", L"Joy%iButton4",
                               L"Joy%iUpValue", L"Joy%iDownValue", L"Joy%iLeftValue", L"Joy%iRightValue"};

enum joyKeyIds {JOY_UP=0, JOY_DOWN, JOY_LEFT, JOY_RIGHT, 
                JOY_B1, JOY_B2, JOY_B3, JOY_B4, 
                JOY_UP_VAL, JOY_DOWN_VAL, JOY_LEFT_VAL, JOY_RIGHT_VAL};

int joyKeyVars[2][12] = {{'W', 'S', 'A', 'D', 'Z', 'X', 'C', 'V', 65, -65, -65, 65},
                         {VK_UP, VK_DOWN, VK_LEFT, VK_RIGHT, VK_NUMPAD0, VK_NUMPAD1, VK_NUMPAD2, VK_NUMPAD3, 65, -65, -65, 65}};

int g_audio_mute = 0;

enum {ASPECT_11=0, ASPECT_43, ASPECT_WIN};
int g_aspect_ratio = ASPECT_11; // 0=1:1, 1=4:3, 2=stretch to window

// various forward declarations
void set_window_title(HWND hwnd);
void write_settings();
bool read_settings(int *port, int *baud);
void find_com_ports(HWND hwnd);

// --------------------------------------------------- Audio ---------------------------------------------------------


#define AUDIOBUFFER_SIZE 0x0400 // must be a power of 2

HANDLE audio_mutex = INVALID_HANDLE_VALUE;
unsigned int  g_audio_sample_ctr = 0;
unsigned int  g_next_audio_sample[2] = {0xffffffff, 0xffffffff};
unsigned char g_next_audio_sample_val[2] = {0, 0};

unsigned int  g_audiobuffer_start[2] = {0, 0}, g_audiobuffer_end[2] = {0, 0};
unsigned int  g_audiobuffer[2][AUDIOBUFFER_SIZE];
#define audiobuffer_empty(N) (g_audiobuffer_start[N]==g_audiobuffer_end[N])
#define audiobuffer_available_for_write(N) (((g_audiobuffer_start[N]+AUDIOBUFFER_SIZE)-g_audiobuffer_end[N]-1)&(AUDIOBUFFER_SIZE-1))

static bool    audio_thread_stop   = false;
static HANDLE  audio_sample_event  = NULL;
static HANDLE  audio_thread_handle = NULL;


void audiobuffer_enqueue(int N, unsigned int b)
{
  WaitForSingleObject(audio_mutex, INFINITE);
  g_audiobuffer[N][g_audiobuffer_end[N]] = b;
  g_audiobuffer_end[N] = (g_audiobuffer_end[N]+1) & (AUDIOBUFFER_SIZE-1);
  ReleaseMutex(audio_mutex);
}

unsigned int audiobuffer_dequeue(int N)
{
  WaitForSingleObject(audio_mutex, INFINITE);
  unsigned int data = g_audiobuffer[N][g_audiobuffer_start[N]];
  g_audiobuffer_start[N] = (g_audiobuffer_start[N]+1) & (AUDIOBUFFER_SIZE-1);
  ReleaseMutex(audio_mutex);
  return data;
}


#define _USE_MATH_DEFINES
#include <math.h>
static DWORD generateTestTone(void *bufferPtr, DWORD numFramesAvailable, UINT nChannels, DWORD sampleRate, double dFreq)
{
  static unsigned long sample = 0;
  short int *dataPtr = (short int *) bufferPtr;

  DWORD periodSamples = (DWORD) ((48000.0 / dFreq) + 0.5);
  double dK = (dFreq * 2.0 * M_PI) / (double) sampleRate;

  for (DWORD i = 0; i < numFramesAvailable; i++)
    {
      // rectangle
      //*dataPtr++ = sample < periodSamples / 2 ? -10000 : 10000;
      //*dataPtr++ = sample < periodSamples / 2 ? -10000 : 10000;
      // sawtooth
      //*dataPtr++ = (short int) ((10000.0 / (double)periodSamples) * (double)sample);
      //*dataPtr++ = (short int) ((10000.0 / (double)periodSamples) * (double)sample);
      // sine
      *dataPtr++ = (short int) (20000 * sin((double)sample * dK));
      *dataPtr++ = (short int) (20000 * sin((double)sample * dK));
      if (++sample == periodSamples) sample = 0;
    }

  return numFramesAvailable;
}


static unsigned long WINAPI audio_thread(HANDLE init_signal)
{
  const GUID PcmSubformatGuid         = { STATIC_KSDATAFORMAT_SUBTYPE_PCM };
  const GUID IID_IAudioClient         = { 0x1CB9AD4C, 0xDBFA, 0x4c32, 0xB1, 0x78, 0xC2, 0xF5, 0x68, 0xA7, 0x03, 0xB2 };
  const GUID IID_IAudioRenderClient   = { 0xF294ACFC, 0x3146, 0x4483, 0xA7, 0xBF, 0xAD, 0xDC, 0xA7, 0xC2, 0x60, 0xE2 };
  const GUID CLSID_MMDeviceEnumerator = { 0xBCDE0395, 0xE52F, 0x467C, 0x8E, 0x3D, 0xC4, 0x57, 0x92, 0x91, 0x69, 0x2E };
  const GUID IID_IMMDeviceEnumerator  = { 0xA95664D2, 0x9614, 0x4F35, 0xA7, 0x46, 0xDE, 0x8D, 0xB6, 0x36, 0x17, 0xE6 };

  IMMDeviceEnumerator *pEnumerator;
  IMMDevice *iMMDevice;
  IAudioClient *iAudioClient;
  REFERENCE_TIME minDuration;
  WAVEFORMATEXTENSIBLE desiredFormat;
  IAudioRenderClient *iAudioRenderClient;
  HANDLE hTask;
  UINT32 bufferFrameCount;
  BYTE *pData;
  DWORD taskIndex = 0;

  // set up desired wave form
  ZeroMemory(&desiredFormat, sizeof(WAVEFORMATEXTENSIBLE));
  desiredFormat.Format.nChannels = 2;
  desiredFormat.Format.nSamplesPerSec = 48000;
  desiredFormat.Format.wBitsPerSample = 16;
  desiredFormat.Samples.wValidBitsPerSample = desiredFormat.Format.wBitsPerSample;
  desiredFormat.Format.nBlockAlign = desiredFormat.Format.nChannels * (desiredFormat.Format.wBitsPerSample/8);
  desiredFormat.Format.nAvgBytesPerSec = desiredFormat.Format.nSamplesPerSec * desiredFormat.Format.nBlockAlign;
  desiredFormat.Format.wFormatTag = WAVE_FORMAT_EXTENSIBLE;
  desiredFormat.Format.cbSize = 22;
  desiredFormat.dwChannelMask = SPEAKER_FRONT_LEFT | SPEAKER_FRONT_RIGHT;
  CopyMemory(&(desiredFormat.SubFormat), &PcmSubformatGuid, sizeof(GUID));

  CoInitialize(0);
  
  if( S_OK==CoCreateInstance(CLSID_MMDeviceEnumerator, 0, CLSCTX_ALL, IID_IMMDeviceEnumerator, (void **)&pEnumerator) )
    {
      // Get the IMMDevice object of the default audio playback (eRender)
      // device, as chosen by user in Control Panel's "Sounds"
      if( S_OK==pEnumerator->GetDefaultAudioEndpoint(eRender, eMultimedia, &iMMDevice) )
        {
          // Get its IAudioClient (used to set audio format, latency, and start/stop)
          if( S_OK==iMMDevice->Activate(IID_IAudioClient, CLSCTX_ALL, 0, (void **)&iAudioClient) )
            {
              // Set the device to play at the minimum latency
              iAudioClient->GetDevicePeriod(NULL, &minDuration);
              
              // Init the device to desired bit rate and resolution
              if( S_OK == iAudioClient->Initialize(AUDCLNT_SHAREMODE_SHARED, AUDCLNT_STREAMFLAGS_EVENTCALLBACK, minDuration, 0, (WAVEFORMATEX *)&desiredFormat, 0) )
                {
                  // Register the event handle
                  if( S_OK == iAudioClient->SetEventHandle(audio_sample_event) )
                    {
                      // Get the actual size (in sample frames) of the audio buffer
                      iAudioClient->GetBufferSize(&bufferFrameCount);
      
                      // Get the IAudioRenderClient
                      if( S_OK == iAudioClient->GetService(IID_IAudioRenderClient, (void **)&iAudioRenderClient) )
                        {
                          // Fill the buffer with silence before we start the stream
                          if( S_OK==iAudioRenderClient->GetBuffer(bufferFrameCount, &pData) )
                            iAudioRenderClient->ReleaseBuffer(bufferFrameCount, AUDCLNT_BUFFERFLAGS_SILENT);

                          // Ask MMCSS to temporarily boost our thread priority
                          // to reduce glitches while the low-latency stream plays
                          hTask = AvSetMmThreadCharacteristics(L"Pro Audio", &taskIndex);

                          // Start audio playback
                          if( S_OK == iAudioClient->Start() )
                            {
                              // Signal main thread that our initialization is done
                              SetEvent(init_signal);
                              
                              // main playback loop
                              short int    current_v[2] = {0,0}, next_v[2] = {0, 0};
                              unsigned int current_t = 0, next_t[2] = {0xFFFFFFFF, 0xFFFFFFFF};
                              while( true )
                                {
                                  WaitForSingleObject(audio_sample_event, INFINITE);
                                  if( audio_thread_stop ) break;
                                  
                                  // try to get a half of the buffer
                                  DWORD numFrames = bufferFrameCount/2;
                                  if( S_OK == iAudioRenderClient->GetBuffer(numFrames, &pData) )
                                    {
                                      // write sound samples to audio buffer
                                      //generateTestTone(pData, numFrames, 2, desiredFormat.Format.nSamplesPerSec, 440);

                                      short int *buf = (short int *) pData;
                                      for(unsigned int i=0; i<numFrames; i++)
                                        {
                                          for(int channel = 0; channel<2; channel++)
                                            {
                                              if( current_t >= next_t[channel] )
                                                {
                                                  current_v[channel] = next_v[channel];
                                                  
                                                  if (audiobuffer_empty(channel))
                                                    {
                                                      next_t[channel] = 0xffffffff; 
                                                      current_v[channel] = 0;
                                                    }
                                                  else
                                                    {
                                                      unsigned int data = audiobuffer_dequeue(channel);
                                                      next_t[channel] = current_t + (data>>8);
                                                      next_v[channel] = ((char)(data & 255)) * 256;
                                                    }
                                                }
                                              else if( next_t[channel]==0xffffffff && !audiobuffer_empty(channel) )
                                                {
                                                  unsigned int data = audiobuffer_dequeue(channel);
                                                  next_t[channel] = current_t + 750;
                                                  next_v[channel] = ((char)(data & 255)) * 256;
                                                }
                                          
                                              *buf++ = current_v[channel];
                                            }

                                          current_t++;
                                        }
                                      
                                      // Let audio device play it
                                      iAudioRenderClient->ReleaseBuffer(numFrames, 0);
                                    }
                                }
                              
                              iAudioClient->Stop();
                            }
                          
                          // revert thread priority
                          if( hTask!=NULL ) AvRevertMmThreadCharacteristics(hTask);
                          
                          iAudioRenderClient->Release();
                        }
                    }
                }
              
              iAudioClient->Release();
            }
          
          iMMDevice->Release();
        }
      
      pEnumerator->Release();
    }

  CoUninitialize();
  audio_thread_handle = NULL;
  SetEvent(init_signal);
  return 0;
}


static void audio_add_sample(int channel, unsigned short delay_us, byte sample)
{
  if( audio_thread_handle )
    {
      static int remainder[2] = { 0, 0 };

      // convert delay in microseconds to delay in 48k sample frames
      // by dividing by 2.833
      // We output audio sample at 48000Hz or one sample every 20.833 microseconds
      // (round division result to nearest)
      delay_us += remainder[channel];
      int delay_samples = (delay_us * 2000) / 20833;
      delay_samples = (delay_samples / 2) + (delay_samples & 1);

      // keep the rounded-off remainder of microseconds to add to the 
      // next sample so we can stay (mostly) in sync
      remainder[channel] = delay_us - (delay_samples * 20833) / 1000;

      if (delay_samples > 0)
        audiobuffer_enqueue(channel, sample + 256 * delay_samples);
    }
}


static int audio_start(void)
{
  int res = 0;

  if( audio_thread_handle==NULL && !g_audio_mute )
    {
      // Get a signal that the audio thread can use to notify us when its done initializing
      HANDLE init_signal = CreateEvent(0, TRUE, 0, 0);
      if( init_signal!=NULL )
        {
          audio_sample_event = CreateEvent(0, 0, 0, 0);
          audio_mutex = CreateMutex(NULL, FALSE, NULL);
          audio_thread_stop = false;
          
          // Create the audio thread
          audio_thread_handle = CreateThread(0, 0, audio_thread, init_signal, 0, NULL);
          if( audio_thread_handle!=NULL )
            {
              // Wait for the audio thread to indicate its initialization is done
              WaitForSingleObject(init_signal, INFINITE);
            }

          // the audio thread sets audio_thread_handle to NULL if it fails to start up
          if( audio_thread_handle==NULL )
            {
              CloseHandle(audio_sample_event);
              CloseHandle(audio_mutex);
            }
        }
    }
  
  return res;
}


static void audio_stop(void)
{
  if( audio_thread_handle )
    {
      // Signal audio thread to terminate and wait
      audio_thread_stop = true;
      SetEvent(audio_sample_event);
      WaitForSingleObject(audio_thread_handle, INFINITE);
      CloseHandle(audio_sample_event);
      CloseHandle(audio_mutex);
    }
}


// --------------------------------------------------- Video ---------------------------------------------------------


HANDLE video_redraw = INVALID_HANDLE_VALUE;
HANDLE video_mutex = INVALID_HANDLE_VALUE;
double border_topbottom = 0, border_leftright = 0;
double byte_width, byte_height;

__int64 performanceFreq, performanceCount;
ID2D1Factory* pDirect2dFactory = NULL;
ID2D1HwndRenderTarget* pRenderTarget = NULL;
ID2D1SolidColorBrush *brushes_color[16], *brushes_grayscale[16];
ID2D1Bitmap *palette[128];


#define P_PIXEL_SIZE 16
static void draw_palette_pixel(ID2D1BitmapRenderTarget *target, byte pc, int x, int y, int s, byte color)
{
  D2D1_RECT_F r;
  r.left   = (float) (x     * P_PIXEL_SIZE * s);
  r.right  = (float) ((x+1) * P_PIXEL_SIZE * s);
  r.top    = (float) (y     * P_PIXEL_SIZE * s);
  r.bottom = (float) ((y+1) * P_PIXEL_SIZE * s);
  target->FillRectangle(&r, (pc & 0x10) ? (brushes_color[color & 15]) : (brushes_grayscale[color & 15]));
}


static void init_palette(byte pc)
{
  ID2D1BitmapRenderTarget *target;
  if( S_OK == pRenderTarget->CreateCompatibleRenderTarget(D2D1::SizeF(4*P_PIXEL_SIZE*256, 2*P_PIXEL_SIZE), &target) )
    {
      // palette for this graphics mode is not yet initialized => initialize now
      target->BeginDraw();

      if( pc & 0x40 )
        {
          // 4x resolution, common color
          byte color = pc & 0x0f;
          for (int i = 0; i < 256; i++)
            {
              draw_palette_pixel(target, pc, (i*4),   0, 1, i & 0x01 ? color : 0);
              draw_palette_pixel(target, pc, (i*4)+1, 0, 1, i & 0x02 ? color : 0);
              draw_palette_pixel(target, pc, (i*4)+2, 0, 1, i & 0x10 ? color : 0);
              draw_palette_pixel(target, pc, (i*4)+3, 0, 1, i & 0x20 ? color : 0);
              draw_palette_pixel(target, pc, (i*4),   1, 1, i & 0x04 ? color : 0);
              draw_palette_pixel(target, pc, (i*4)+1, 1, 1, i & 0x08 ? color : 0);
              draw_palette_pixel(target, pc, (i*4)+2, 1, 1, i & 0x40 ? color : 0);
              draw_palette_pixel(target, pc, (i*4)+3, 1, 1, i & 0x80 ? color : 0);
            }
        }
      else
        {
          // normal resolution, individual color
          for (int i = 0; i < 256; i++)
            {
              draw_palette_pixel(target, pc, i*2,   0, 2, i & 0x0f);
              draw_palette_pixel(target, pc, i*2+1, 0, 2, i >> 4);
            }
        }

      target->EndDraw();
      target->GetBitmap(&(palette[pc]));
      target->Release();
    }
  else
    palette[pc] = NULL;
}


inline void update_byte(byte pc, int x, int y, int b)
{
  D2D1_RECT_F rsrc, rdst;
  rsrc.left   = (float) (4*P_PIXEL_SIZE*b);
  rsrc.right  = (float) (rsrc.left + 4*P_PIXEL_SIZE);
  rsrc.top    = (float) 0.0;
  rsrc.bottom = (float) 2*P_PIXEL_SIZE;

  rdst.left   = (float) (border_leftright + x * byte_width);
  rdst.right  = (float) (rdst.left + byte_width);
  rdst.top    = (float) (border_topbottom + y * byte_height);
  rdst.bottom = (float) (rdst.top + byte_height);

  pRenderTarget->DrawBitmap(palette[pc], rdst, 1.0, D2D1_BITMAP_INTERPOLATION_MODE_NEAREST_NEIGHBOR, rsrc);
}


static void render_frame_smallmem(byte pc, byte *mem, int xo = 0, int yo = 0)
{
  // render one quadrant (or full screen for small memory)
  int y = yo, x = xo;
  for(int i=0; i<512; i++)
    {
      update_byte(pc, x++, y, *mem++);
      if( (i&0x0f)==0x0f ) { x = xo; y++; }
    }
}


static void render_frame_bigmem(byte pc, byte *mem)
{
  // 2k memory => render the four quadrants
  render_frame_smallmem(pc, mem,        0,  0);
  render_frame_smallmem(pc, mem+0x200, 16,  0);
  render_frame_smallmem(pc, mem+0x400,  0, 32);
  render_frame_smallmem(pc, mem+0x600, 16, 32);
}


static void update_frame()
{
  WaitForSingleObject(video_mutex, INFINITE);
  pRenderTarget->BeginDraw();

  if( WaitForSingleObject(video_redraw, 0)==WAIT_TIMEOUT )
    {
      // nothing has changed since last time we rendered the frame
    }
  else if( dazzler_ctrl & 0x80 )
    {
      // make copy of memory and graphics mode 
      // so updates while drawing don't affect the rendering
      byte mem[2048], pc = dazzler_picture_ctrl;
      memcpy(mem, dazzler_mem + 2048 * (dazzler_ctrl & 1), 2048);
      
      // determine on-screen pixel size of one memory byte (4x2 pixels per byte)
      // if using small memory then scale up pixel size by factor 2
      bool bigmem = pc & 0x20;
      byte_width  = P_PIXEL_SIZE * (bigmem ? 4 : 8);
      byte_height = P_PIXEL_SIZE * (bigmem ? 2 : 4);

      // make sure the palette for this graphics mode is initialized
      if( palette[pc]==NULL ) init_palette(pc);

      // render screen memory (only draw if palette is initialized)
	  pRenderTarget->Clear(D2D1::ColorF(D2D1::ColorF::Black));
	  if( palette[pc]==NULL )
        {}
      else if (pc & 0x20)
        render_frame_bigmem(pc, mem);
      else
        render_frame_smallmem(pc, mem);
    }
  else
	pRenderTarget->Clear(D2D1::ColorF(D2D1::ColorF::Black));


  pRenderTarget->EndDraw();
  ReleaseMutex(video_mutex);
}


static unsigned long WINAPI video_thread(void *h)
{
  LARGE_INTEGER ctr1, ctr2;
  QueryPerformanceFrequency(&ctr1);
  performanceFreq = ctr1.QuadPart;
  QueryPerformanceCounter(&ctr1);
  
  while( true )
    {
      // render screen
      update_frame();

      // measure time (for fps display)
      QueryPerformanceCounter(&ctr2);
      if (performanceCount == 0)
        performanceCount = ctr2.QuadPart - ctr1.QuadPart;
      else
        performanceCount = (performanceCount*3 + ctr2.QuadPart-ctr1.QuadPart)/4;
      ctr1 = ctr2;
   }
}


static void video_start(HWND hwnd)
{
  if (S_OK == D2D1CreateFactory(D2D1_FACTORY_TYPE_MULTI_THREADED, &pDirect2dFactory))
    {
      RECT rc;
      GetClientRect(hwnd, &rc);
      UINT w = rc.right - rc.left, h = rc.bottom - rc.top;

      if ( S_OK==pDirect2dFactory->CreateHwndRenderTarget(D2D1::RenderTargetProperties(), D2D1::HwndRenderTargetProperties(hwnd, D2D1::SizeU(w, h)), &pRenderTarget))
        {
          // set initial pixel scaling
          pRenderTarget->SetTransform(D2D1::Matrix3x2F::Identity());

          // initialize color brushes
          for(int i = 0; i < 16; i++)
            {
              pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(RGB2BGR(colors[i])), &(brushes_color[i]));
              pRenderTarget->CreateSolidColorBrush(D2D1::ColorF(RGB(17*i,17*i,17*i)), &(brushes_grayscale[i]));
            }

          // set all palette pointers to "not initialized"
          memset(palette, 0, 128 * sizeof(ID2D1Bitmap *));
          
          // initialize mutex (necessary since WM_SIZE and video thread can not
          // access the renterTarget simultaneously)
          video_mutex  = CreateMutex(NULL, FALSE, NULL);
          
          // initialize event to signal that video needs to be redrawn
          video_redraw = CreateEvent(0, 0, 0, 0);
          
          // Create the video thread
          HANDLE h = CreateThread(0, 0, video_thread, hwnd, 0, NULL);
          CloseHandle(h);
        }
    }
}


// --------------------------------------------------- Communication ---------------------------------------------------------


// serial port communication
int    g_com_port = -1;
int    g_com_baud = 1050000;
HANDLE serial_conn = INVALID_HANDLE_VALUE;

// socket communication
wchar_t *peer = NULL;
SOCKET server_socket = INVALID_SOCKET;


void dazzler_send(HWND hwnd, byte *data, int size)
{
  DWORD n;

  if( serial_conn!=INVALID_HANDLE_VALUE )
    WriteFile(serial_conn, data, size, &n, NULL);
  if( server_socket!=INVALID_SOCKET )
    send(server_socket, (char *) data, size, 0);
}


void dazzler_receive(HWND hwnd, byte *data, int size)
{
  static int recv_status = 0, recv_bytes = 0, recv_ptr = 0;
  static byte buf[10];

  int i = 0;
  while( i<size )
    {
      if( recv_bytes>0 )
        {
          int n = recv_bytes > (size-i) ? (size-i) : recv_bytes;

          if( recv_status==DAZ_FULLFRAME )
            memcpy(dazzler_mem+recv_ptr, data+i, n);
          else
            memcpy(buf+recv_ptr, data+i, n);

          recv_bytes -= n;
          recv_ptr   += n;
          i          += n;

          if( recv_bytes == 0 )
            {
              switch( recv_status )
                {
                case DAZ_MEMBYTE:
                  {
                    int a = buf[0]*256+buf[1];
                    dazzler_mem[a] = buf[2];
                    SetEvent(video_redraw);
                    break;
                  }

                case DAZ_DAC:
                  {
                    audio_add_sample(buf[0] == 0 ? 0 : 1, buf[1] + buf[2] * 256, buf[3]);
                    break;
                  }

                case DAZ_CTRL:
                  {
                    // computer version 0 only supports a single buffer but bit 0
                    // may be on or off
                    if (computer_version < 1) buf[0] &= 0x80;

                    if( (dazzler_ctrl&0x81) != (buf[0]&0x81) )
                      {
                        // only redraw title if on/off status has changed
                        bool setTitle = (dazzler_ctrl&0x80) != (buf[0]&0x80);
                        dazzler_ctrl = buf[0];
                        if( setTitle ) set_window_title(hwnd);
                        SetEvent(video_redraw);
                      }
                    break;
                  }

                case DAZ_CTRLPIC:
                  {
                    // bit 7 is unused, bits 0-3 (color) are only used if bit 6 (high-res) is set
                    buf[0] = (buf[0] & 0x40) ? (buf[0] & 0x7f) : (buf[0] & 0x70);
                    if( buf[0]!=dazzler_picture_ctrl ) 
                      { 
                        dazzler_picture_ctrl=buf[0]; 
                        SetEvent(video_redraw); 
                      }
                    break;
                  }

                case DAZ_FULLFRAME:
                  SetEvent(video_redraw);
                  break;
                }

              recv_status = 0;
            }
        }
      else
        {
          recv_status = data[i] & 0xf0;
          recv_bytes = 0;
          recv_ptr   = 0;

          switch( recv_status )
            {
            case DAZ_MEMBYTE:
              recv_bytes = 2;
              buf[recv_ptr++] = data[i]&0x0F;
              break;

            case DAZ_DAC:
              recv_bytes = 3;
              buf[recv_ptr++] = data[i]&0x0F;
              break;

            case DAZ_CTRL:
            case DAZ_CTRLPIC:
              recv_bytes = 1;
              break;

            case DAZ_VERSION:
              {
                computer_version = data[i] & 0x0F;
                
                // respond by sending our version to the computer
                unsigned char b[3];
                b[0] = DAZ_VERSION | (DAZZLER_VERSION&0x0F);
                b[1] = FEAT_VIDEO | FEAT_DUAL_BUF | FEAT_JOYSTICK | FEAT_KEYBOARD | FEAT_DAC;
                b[2] = 0;
                
                // only computer version 2 or later expects feature information
                // (computer version 0 does not send DAZ_VERSION)
                dazzler_send(hwnd, b, computer_version<2 ? 1 : 3);
                recv_status = 0;
                break;
              }

            case DAZ_FULLFRAME: 
              recv_bytes = (data[i] & 0x01) ? 2048 : 512;
              recv_ptr   = (data[i] & 0x08) * 256;
              break;

            default:
              recv_status = 0;
              break;
            }

          i++;
        }
    }
}


SOCKET connect_socket(HWND hwnd, const wchar_t *server)
{
  SOCKET ConnectSocket = INVALID_SOCKET;
  WSADATA wsaData;
  int iResult;
  ADDRINFOW *result = NULL, *ptr = NULL, hints;
    
  // Initialize Winsock
  iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
  if (iResult != 0) {
    return INVALID_SOCKET;
  }

  ZeroMemory( &hints, sizeof(hints) );
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = IPPROTO_TCP;

  // Resolve the server address and port
  iResult = GetAddrInfo(server, L"8800", &hints, &result);
  if ( iResult != 0 ) {
    WSACleanup();
    return INVALID_SOCKET;
  }

  // Attempt to connect to an address until one succeeds
  for(ptr=result; ptr != NULL ;ptr=ptr->ai_next) 
    {
      // Create a SOCKET for connecting to server
      ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype, ptr->ai_protocol);
      if (ConnectSocket == INVALID_SOCKET) {
        WSACleanup();
        return INVALID_SOCKET;
      }
        
      // Connect to server.
      iResult = connect( ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
      if (iResult == SOCKET_ERROR) 
        {
          closesocket(ConnectSocket);
          ConnectSocket = INVALID_SOCKET;
          continue;
        }
      break;
    }

  FreeAddrInfo(result);

  if (ConnectSocket == INVALID_SOCKET) 
    {
      WSACleanup();
      return INVALID_SOCKET;
    }
    
  return ConnectSocket;
}


DWORD WINAPI serial_thread(void *data)
{
  const char *portName = (const char *) data;
  DWORD dwRead;
  HWND hwnd = (HWND) data;
  int current_port = -1, current_baud = -1;
 
  while( true )
    {
      if( (g_com_port!=current_port || g_com_baud!=current_baud) && serial_conn!=INVALID_HANDLE_VALUE )
        {
          // we are connected and the port has changed => disconnect
          CloseHandle(serial_conn);
          serial_conn=INVALID_HANDLE_VALUE;
          set_window_title(hwnd);
        }

      if( serial_conn==INVALID_HANDLE_VALUE )
        {
          find_com_ports(hwnd);

          if( g_com_port>0 )
            {
              wchar_t comName[100];
              wsprintf(comName, L"\\\\.\\COM%i", g_com_port);
              serial_conn = CreateFile(comName, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
              if( serial_conn!=INVALID_HANDLE_VALUE )
                {
                  DCB dcb;
                  FillMemory(&dcb, sizeof(dcb), 0);
                  if( GetCommState(serial_conn, &dcb) )
                    {
                      dcb.ByteSize = 8;
                      dcb.Parity   = NOPARITY;
                      dcb.StopBits = ONESTOPBIT;
                      dcb.BaudRate = g_com_baud;
                      SetCommState(serial_conn, &dcb);
                    }

                  COMMTIMEOUTS timeouts;
                  timeouts.ReadIntervalTimeout = 5; 
                  timeouts.ReadTotalTimeoutMultiplier = 0;
                  timeouts.ReadTotalTimeoutConstant = 5;
                  timeouts.WriteTotalTimeoutMultiplier = 0;
                  timeouts.WriteTotalTimeoutConstant = 0;
                  SetCommTimeouts(serial_conn, &timeouts);
                  set_window_title(hwnd);
                  current_port = g_com_port;
                  current_baud = g_com_baud;
                }
              else
                Sleep(200);
            }
          else
            Sleep(500);
        }
      else
        {
          byte buf[100];
          if( ReadFile(serial_conn, buf, 100, &dwRead, NULL) )
            {
              if( dwRead>0 ) dazzler_receive(hwnd, buf, dwRead);
            }
          else
            {
              CloseHandle(serial_conn);
              serial_conn=INVALID_HANDLE_VALUE;
              set_window_title(hwnd);
            }
        }
    }
}


// ---------------------------------------------------- Main Window ----------------------------------------------------------


enum
{
  ID_SOCKET = WM_USER,
  ID_FILE_EXIT,
  ID_VIEW_FULLSCREEN,
  ID_VIEW_NORMAL,
  ID_VIEW_ASPECT_11,
  ID_VIEW_ASPECT_43,
  ID_VIEW_ASPECT_WIN,
  ID_SETTINGS_JOY_SWAP,
  ID_SETTINGS_JOY_SHOW,
  ID_SETTINGS_JOY_KEYS,
  ID_SETTINGS_AUDIO_MUTE,
  ID_SETTINGS_BAUD_9600,
  ID_SETTINGS_BAUD_38400,
  ID_SETTINGS_BAUD_115200,
  ID_SETTINGS_BAUD_250000,
  ID_SETTINGS_BAUD_525000,
  ID_SETTINGS_BAUD_750000,
  ID_SETTINGS_BAUD_1050000,
  ID_SETTINGS_PORT_NONE,
  ID_SETTINGS_PORT,
  ID_HELP_ABOUT,
  };


void set_baud_rate(HWND hwnd, int baud)
{
  int id;
  if( baud<=9600         ) id = ID_SETTINGS_BAUD_9600;
  else if( baud<=38400   ) id = ID_SETTINGS_BAUD_38400;
  else if( baud<=115200  ) id = ID_SETTINGS_BAUD_115200;
  else if( baud<=250000  ) id = ID_SETTINGS_BAUD_250000;
  else if( baud<=525000  ) id = ID_SETTINGS_BAUD_525000;
  else if( baud<=750000  ) id = ID_SETTINGS_BAUD_750000;
  else                     id = ID_SETTINGS_BAUD_1050000;

  g_com_baud = baud;
  HMENU menuBaud = GetSubMenu(GetSubMenu(GetMenu(hwnd), 2), 1);
  CheckMenuRadioItem(menuBaud, ID_SETTINGS_BAUD_9600, ID_SETTINGS_BAUD_1050000, id, MF_BYCOMMAND);
  write_settings();
}


void set_com_port(HWND hwnd, int port)
{
  g_com_port = port;

  HMENU menuPort = GetSubMenu(GetSubMenu(GetMenu(hwnd), 2), 0);
  if( !CheckMenuRadioItem(menuPort, ID_SETTINGS_PORT-1, ID_SETTINGS_PORT+255, ID_SETTINGS_PORT+g_com_port, MF_BYCOMMAND) )
    {
      // checking the item failed => no such port
      g_com_port = -1;
      CheckMenuRadioItem(menuPort, ID_SETTINGS_PORT-1, ID_SETTINGS_PORT+255, ID_SETTINGS_PORT+g_com_port, MF_BYCOMMAND);
    }

  set_window_title(hwnd);
  if( g_com_port>0 ) write_settings();
}


void find_com_ports(HWND hwnd)
{
  static bool known_port[256], first_run = true;
  bool found_port[256];
  HKEY key;

  if( first_run ) memset(known_port, 0, sizeof(known_port));
  memset(found_port, 0, sizeof(found_port));

  if(RegOpenKeyEx(HKEY_LOCAL_MACHINE, L"HARDWARE\\DEVICEMAP\\SERIALCOMM", 0, KEY_ENUMERATE_SUB_KEYS|KEY_QUERY_VALUE, &key) == ERROR_SUCCESS)
    {
      DWORD   dwIndex = 0, dwType, lpcName = 256, lpcValue = 256;
      wchar_t lpName[256], lpValue[256];

      while( RegEnumValue(key, dwIndex++, lpName, &lpcName, NULL, &dwType, (LPBYTE) lpValue, &lpcValue) == ERROR_SUCCESS )
        {
          lpValue[lpcValue]=0;
          if( wcsncmp(lpValue, L"COM", 3)==0 )
            {
              int i = _wtoi(lpValue+3);
              if( i<256 ) found_port[i] = true;
            }

          lpcName = 256;
          lpcValue = 256;
        }

      RegCloseKey(key);
    }

  // check whether any port changes were detected
  int i = 0;
  if( !first_run )
    for(i=0; i<256; i++)
      if( found_port[i]!=known_port[i] )
        break;

  if( i<256 )
    {
      // port change was detected => rebuild "Ports" menu
      int new_port = -1;
      HMENU menuPort = CreateMenu();
      AppendMenu(menuPort, MF_BYPOSITION | MF_STRING, ID_SETTINGS_PORT_NONE, L"None");
      for(int i=0; i<256; i++)
        {
          if( found_port[i] )
            {
              // check if port "i" was newly found in this check
              if( !known_port[i] )
                {
                  // port did not exist before
                  if( new_port==-1 )
                    new_port = i;
                  else if( new_port>0 )
                    new_port = -2;
                }

              wchar_t comName[10];
              wsprintf(comName, L"COM%i", i);
              AppendMenu(menuPort, MF_BYPOSITION | MF_STRING, ID_SETTINGS_PORT+i, comName);
            }

          known_port[i] = found_port[i];
        }
  
      HMENU menuSettings = GetSubMenu(GetMenu(hwnd), 2);
      ModifyMenu(menuSettings, 0, MF_BYPOSITION|MF_POPUP, (UINT_PTR) menuPort, L"&Port");
      if( !first_run && new_port>0 ) 
        set_com_port(hwnd, new_port);
      else
        CheckMenuRadioItem(menuPort, ID_SETTINGS_PORT-1, ID_SETTINGS_PORT+255, ID_SETTINGS_PORT+g_com_port, MF_BYCOMMAND);
    }

  first_run = false;
}


void set_window_title(HWND hwnd)
{
  bool connected = (serial_conn!=INVALID_HANDLE_VALUE) || (server_socket!=INVALID_SOCKET);
  bool on = (dazzler_ctrl & 0x80)!=0;
  wchar_t buf[100];

  int fps = performanceCount==0 ? 0 : (int) ((((double) performanceFreq)/((double) performanceCount)) + 0.5);
  
  if( peer!=NULL )
    wsprintf(buf, L"Dazzler Display (%s, %sconnected, %s, %i fps)",
             peer, connected ? L"" : L"not ", on ? L"on" : L"off", fps);
  else if( g_com_port>0 )
    wsprintf(buf, L"Dazzler Display (COM%i, %sconnected, %s, %i fps)",
             g_com_port, connected ? L"" : L"not ", on ? L"on" : L"off", fps);
  else
    wsprintf(buf, L"Dazzler Display");

  if( g_joy_show )
    {
      wchar_t buf2[100];
      wsprintf(buf2, L" --- J1:%s%s%s%s%s%s J2:%s%s%s%s%s%s",
               (char) g_joy1[2]>32 ? L"U" : (char) g_joy1[2]<-32 ? L"D" : L"",
               (char) g_joy1[1]>32 ? L"R" : (char) g_joy1[1]<-32 ? L"L" : L"",
               g_joy1[0] & 0x01 ? L"" : L"1", g_joy1[0] & 0x02 ? L"" : L"2", g_joy1[0] & 0x04 ? L"" : L"3", g_joy1[0] & 0x08 ? L"" : L"4",
               (char) g_joy2[2]>32 ? L"U" : (char) g_joy2[2]<-32 ? L"D" : L"",
               (char) g_joy2[1]>32 ? L"R" : (char) g_joy2[1]<-32 ? L"L" : L"",
               g_joy2[0] & 0x01 ? L"" : L"1", g_joy2[0] & 0x02 ? L"" : L"2", g_joy2[0] & 0x04 ? L"" : L"3", g_joy2[0] & 0x08 ? L"" : L"4");
      wcscat_s(buf, buf2);
    }

  SetWindowText(hwnd, buf);
}


void write_settings()
{
  HKEY key;
  if( RegCreateKeyEx(HKEY_CURRENT_USER, L"Software\\DazzlerDisplay", 0, NULL, REG_OPTION_NON_VOLATILE, KEY_SET_VALUE, NULL, &key, NULL) == ERROR_SUCCESS )
    {
      RegSetValueEx(key, L"Port", 0, REG_DWORD, (const LPBYTE) &g_com_port, 4);
      RegSetValueEx(key, L"Baud", 0, REG_DWORD, (const LPBYTE) &g_com_baud, 4);
      RegSetValueEx(key, L"SwapJoysticks", 0, REG_DWORD, (const LPBYTE) &g_joy_swap, 4);
      RegSetValueEx(key, L"ShowJoysticks", 0, REG_DWORD, (const LPBYTE) &g_joy_show, 4);
      RegSetValueEx(key, L"JoystickKeys", 0, REG_DWORD, (const LPBYTE) &g_joy_keys, 4);
      RegSetValueEx(key, L"MuteAudio", 0, REG_DWORD, (const LPBYTE) &g_audio_mute, 4);
      RegSetValueEx(key, L"AspectRatio", 0, REG_DWORD, (const LPBYTE) &g_aspect_ratio, 4);
      RegCloseKey(key);
    }
}


bool read_settings(int *port, int *baud)
{
  bool res = false;

  DWORD l = 4, tp;
  HKEY key;
  if( RegOpenKeyEx(HKEY_CURRENT_USER, L"Software\\DazzlerDisplay", 0, KEY_QUERY_VALUE, &key) == ERROR_SUCCESS )
    {
      RegQueryValueEx(key, L"Port", 0, &tp, (LPBYTE) port, &l);
      RegQueryValueEx(key, L"Baud", 0, &tp, (LPBYTE) baud, &l);
      RegQueryValueEx(key, L"SwapJoysticks", 0, &tp, (LPBYTE) &g_joy_swap, &l);
      RegQueryValueEx(key, L"ShowJoysticks", 0, &tp, (LPBYTE) &g_joy_show, &l);
      RegQueryValueEx(key, L"JoystickKeys", 0, &tp, (LPBYTE) &g_joy_keys, &l);
      RegQueryValueEx(key, L"MuteAudio", 0, &tp, (LPBYTE) &g_audio_mute, &l);
      RegQueryValueEx(key, L"AspectRatio", 0, &tp, (LPBYTE) &g_aspect_ratio, &l);
      RegCloseKey(key);
      
      if( RegCreateKeyEx(HKEY_CURRENT_USER, L"Software\\DazzlerDisplay\\JoyKeys", 0, NULL, REG_OPTION_NON_VOLATILE, KEY_SET_VALUE|KEY_QUERY_VALUE, NULL, &key, NULL) == ERROR_SUCCESS )
        {
          wchar_t name[100];
          for(int i=0; i<2; i++)
            {
              for(int j=0; j<12; j++)
                {
                  wsprintf(name, joyKeyRegNames[j], i+1);
                  if( RegQueryValueEx(key, name, 0, &tp, (LPBYTE) &(joyKeyVars[i][j]), &l) != ERROR_SUCCESS )
                    RegSetValueEx(key, name, 0, REG_DWORD, (const LPBYTE) &(joyKeyVars[i][j]), 4);
                }
            }
        }

      res = true;
    }

  return res;
}


void calc_window_size(long *w, long *h)
{
  RECT r;
  r.left = 0;
  r.right = *w;
  r.top = 0;
  r.bottom = *h;
  AdjustWindowRectEx(&r, WS_OVERLAPPEDWINDOW, true, 0);
  *w = r.right-r.left;
  *h = r.bottom-r.top;

  // if aspect ratio is 4:3 then make window wider
  if( g_aspect_ratio == ASPECT_43 ) *w = (*w * 4) / 3;
}


void adjust_render_area_size(HWND hwnd)
{
  if( pRenderTarget != NULL )
    {
      WaitForSingleObject(video_mutex, INFINITE);
      
      if( g_aspect_ratio==ASPECT_WIN )
        {
          // aspect ratio is set to "stretch" so set a virtual
          // width/height of 128 pixels regardless of window size
          pRenderTarget->Resize(D2D1::SizeU(P_PIXEL_SIZE*128, P_PIXEL_SIZE*128));
          border_topbottom = border_leftright = 0;
        }
      else
        {
          // get window (client) size
          RECT r;
          GetClientRect(hwnd, &r);
          int width  = r.right-r.left;
          int height = r.bottom-r.top;

          // if aspect ratio is 4:3 then widen pixels by reducing 
          // virtual width ov window
          if (g_aspect_ratio == ASPECT_43) width = (width * 3) / 4;

          if( width>0 && height>0)
            {
              // fix maximum virtual width or height to 128 pixels and scale 
              // other dimension according to current window width:height ratio
              // this will scale up the pixel size
              if (width < height)
                {
                  height = (height * P_PIXEL_SIZE * 128) / width;
                  width = P_PIXEL_SIZE * 128;
                }
              else
                {
                  width = (width * P_PIXEL_SIZE * 128) / height;
                  height = P_PIXEL_SIZE * 128;
                }
            }
              
          // set virtual width of render area
          pRenderTarget->Resize(D2D1::SizeU(width, height));

          // set borders to center picture
          border_topbottom = height > width  ? (height-width)/2: 0;
          border_leftright = width  > height ? (width-height)/2: 0;
        }
      
      ReleaseMutex(video_mutex);
      SetEvent(video_redraw);
    }
}


void toggle_fullscreen(HWND hwnd)
{
  static WINDOWPLACEMENT s_wpPrev = { sizeof(s_wpPrev) };
  static HMENU s_menu = NULL;

  DWORD dwStyle = GetWindowLong(hwnd, GWL_STYLE);
  if( dwStyle & WS_OVERLAPPEDWINDOW )
    {
      // turn full-screen mode on
      MONITORINFO mi = { sizeof(mi) };
      s_menu = GetMenu(hwnd);
      SetMenu(hwnd, NULL);
      if( GetWindowPlacement(hwnd, &s_wpPrev) && GetMonitorInfo(MonitorFromWindow(hwnd, MONITOR_DEFAULTTOPRIMARY), &mi) )
        {
          SetWindowLong(hwnd, GWL_STYLE, dwStyle & ~WS_OVERLAPPEDWINDOW);
          SetWindowPos(hwnd, 
                       HWND_TOP,mi.rcMonitor.left, mi.rcMonitor.top,
                       mi.rcMonitor.right - mi.rcMonitor.left,
                       mi.rcMonitor.bottom - mi.rcMonitor.top,
                       SWP_NOOWNERZORDER | SWP_FRAMECHANGED);
          set_window_title(hwnd);
        }
    } 
  else 
    {
      // turn full-screen mode off
      SetWindowLong(hwnd, GWL_STYLE,dwStyle | WS_OVERLAPPEDWINDOW);
      SetWindowPlacement(hwnd, &s_wpPrev);
      SetWindowPos(hwnd, NULL, 0, 0, 0, 0,SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_NOOWNERZORDER | SWP_FRAMECHANGED);
      SetMenu(hwnd, s_menu);
    }
}


char get_joy_value(int v)
{
  v = (v-0x8000)/256;

  // Some games can have problems if the joystick values 
  // go all the way to extremes -128/127. For example there seems to be
  // a bug in Gotcha where if the initial value read for the joystick 
  // is 0 then a full left or full down (value -128) will actually move
  // the player in the opposite direction.
  // So we keep the values in range -127..126 to avoid that.
  if( v>=-4 && v<=4)
    return 0;
  else if( v<-127 )
    return -127;
  else if( v>126 )
    return 126;
  else
    return v;
}


LRESULT CALLBACK WindowProc(HWND hwnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
  switch (uMsg)
    {
    case WM_COMMAND: 
      {
        int id = LOWORD(wParam);

        // Test for the identifier of a command item. 
        switch( id )
          { 
          case ID_FILE_EXIT: 
            PostQuitMessage(0); 
            break;

          case ID_VIEW_FULLSCREEN: 
            toggle_fullscreen(hwnd); 
            break;

          case ID_VIEW_NORMAL:
            {
              // if full-screen mode is on then turn it off
              if( !(GetWindowLong(hwnd, GWL_STYLE) & WS_OVERLAPPEDWINDOW) ) toggle_fullscreen(hwnd);

              // reset window to initial size
              long w = 128*4, h = 128*4;
              calc_window_size(&w, &h);
              SetWindowPos(hwnd, 0, 0, 0, w, h, SWP_NOMOVE | SWP_NOOWNERZORDER);
              break;
            }

          case ID_VIEW_ASPECT_11:
          case ID_VIEW_ASPECT_43:
          case ID_VIEW_ASPECT_WIN:
            {
              g_aspect_ratio = id-ID_VIEW_ASPECT_11;
              CheckMenuRadioItem(GetSubMenu(GetMenu(hwnd), 1), ID_VIEW_ASPECT_11, ID_VIEW_ASPECT_WIN, ID_VIEW_ASPECT_11+g_aspect_ratio, MF_BYCOMMAND);
              adjust_render_area_size(hwnd);
              write_settings();
              break;
            }

          case ID_SETTINGS_BAUD_9600:    set_baud_rate(hwnd, 9600); break;
          case ID_SETTINGS_BAUD_38400:   set_baud_rate(hwnd, 38400); break;
          case ID_SETTINGS_BAUD_115200:  set_baud_rate(hwnd, 115200); break;
          case ID_SETTINGS_BAUD_250000:  set_baud_rate(hwnd, 250000); break;
          case ID_SETTINGS_BAUD_525000:  set_baud_rate(hwnd, 525000); break;
          case ID_SETTINGS_BAUD_750000:  set_baud_rate(hwnd, 750000); break;
          case ID_SETTINGS_BAUD_1050000: set_baud_rate(hwnd, 1050000); break;

          case ID_SETTINGS_JOY_SWAP:
            {
              g_joy_swap = !g_joy_swap;
              CheckMenuItem(GetSubMenu(GetMenu(hwnd), 2), ID_SETTINGS_JOY_SWAP, MF_BYCOMMAND | (g_joy_swap ? MF_CHECKED : MF_UNCHECKED));
              write_settings();

              byte b[2];
              memcpy(b, g_joy1+1, 2); memcpy(g_joy1+1, g_joy2+1, 2); memcpy(g_joy2+1, b, 2);
              dazzler_send(hwnd, g_joy1, 3);
              dazzler_send(hwnd, g_joy2, 3);
              if( g_joy_show ) set_window_title(hwnd);
              break;
            }

          case ID_SETTINGS_JOY_KEYS:
            {
              g_joy_keys = !g_joy_keys;
              CheckMenuItem(GetSubMenu(GetMenu(hwnd), 2), ID_SETTINGS_JOY_KEYS, MF_BYCOMMAND | (g_joy_keys ? MF_CHECKED : MF_UNCHECKED));
              write_settings();
              break;
            }

          case ID_SETTINGS_JOY_SHOW:
            {
              g_joy_show = !g_joy_show;
              CheckMenuItem(GetSubMenu(GetMenu(hwnd), 2), ID_SETTINGS_JOY_SHOW, MF_BYCOMMAND | (g_joy_show ? MF_CHECKED : MF_UNCHECKED));
              write_settings();
              set_window_title(hwnd);
              break;
            }

          case ID_SETTINGS_AUDIO_MUTE:
            {
              g_audio_mute = !g_audio_mute;
              if( g_audio_mute ) audio_stop(); else audio_start();
              CheckMenuItem(GetSubMenu(GetMenu(hwnd), 2), ID_SETTINGS_AUDIO_MUTE, MF_BYCOMMAND | (g_audio_mute ? MF_CHECKED : MF_UNCHECKED));
              write_settings();
              break;
            }

          case ID_HELP_ABOUT:
            MessageBox(hwnd, 
                       L"Cromemco Dazzler Display application for\nArduino Altair 88000 simulator\n\n"
                       L"https://www.hackster.io/david-hansel/dazzler-display-for-altair-simulator-3febc6\n"
                       L"https://www.hackster.io/david-hansel/arduino-altair-8800-simulator-3594a6\n\n"
                       L"(C) 2018-2019 David Hansel", 
                       L"About", MB_OK | MB_ICONINFORMATION);
            break;

          default:
            {
              if( id>=ID_SETTINGS_PORT_NONE && id<ID_SETTINGS_PORT+256 )
                set_com_port(hwnd, id-ID_SETTINGS_PORT);
              break;
            }
          }
        break;
      }

    case WM_SIZE:
      {
        adjust_render_area_size(hwnd);
        break;
      }
      
    case WM_CHAR: 
      {
        if( wParam==('F'-64) || (wParam==27 && (GetWindowLong(hwnd, GWL_STYLE)&WS_OVERLAPPEDWINDOW)==0) )
          toggle_fullscreen(hwnd);
        else if( wParam==('J'-64) )
          WindowProc(hwnd, WM_COMMAND, ID_SETTINGS_JOY_SWAP, 0);
        else if( wParam==('N'-64) )
          WindowProc(hwnd, WM_COMMAND, ID_VIEW_NORMAL, 0);

        if( !g_joy_keys )
          {
            byte msg[2];
            msg[0] = DAZ_KEY;
            msg[1] = wParam;
            dazzler_send(hwnd, msg, 2);
          }

        break;
      }
      
    case WM_KEYDOWN:
      {
        if( g_joy_keys )
          {
            int joykey = 0;
            byte *joy[2];
            joy[0] = g_joy_swap ? g_joy2 : g_joy1;
            joy[1] = g_joy_swap ? g_joy1 : g_joy2;
            
            for(int i=0; i<2 && joykey==0; i++)
              {
                joykey = i+1;
                if     ( wParam==joyKeyVars[i][JOY_UP] )    joy[i][2] = joyKeyVars[i][JOY_UP_VAL];
                else if( wParam==joyKeyVars[i][JOY_DOWN] )  joy[i][2] = joyKeyVars[i][JOY_DOWN_VAL];
                else if( wParam==joyKeyVars[i][JOY_LEFT] )  joy[i][1] = joyKeyVars[i][JOY_LEFT_VAL];
                else if( wParam==joyKeyVars[i][JOY_RIGHT] ) joy[i][1] = joyKeyVars[i][JOY_RIGHT_VAL];
                else if( wParam==joyKeyVars[i][JOY_B1] )    joy[i][0] &= ~1;
                else if( wParam==joyKeyVars[i][JOY_B2] )    joy[i][0] &= ~2;
                else if( wParam==joyKeyVars[i][JOY_B3] )    joy[i][0] &= ~4;
                else if( wParam==joyKeyVars[i][JOY_B4] )    joy[i][0] &= ~8;
                else joykey = 0;
              }
            
            if( joykey )
              {
                dazzler_send(hwnd, ((joykey==1)^g_joy_swap) ? g_joy1 : g_joy2, 3);
                if (g_joy_show) set_window_title(hwnd);
              }
            else if( wParam==' ' ) 
              {
                // space bar is button 1 for both joysticks
                g_joy1[0] &= ~1;
                g_joy2[0] &= ~1;
                dazzler_send(hwnd, g_joy1, 3);
                dazzler_send(hwnd, g_joy2, 3);
                if( g_joy_show ) set_window_title(hwnd);
              }
          }

        break;
      }

    case WM_KEYUP:
      {
        if( g_joy_keys )
          {
            int joykey = 0;
            byte *joy[2];
            joy[0] = g_joy_swap ? g_joy2 : g_joy1;
            joy[1] = g_joy_swap ? g_joy1 : g_joy2;
            
            for(int i=0; i<2 && joykey==0; i++)
              {
                joykey = i+1;
                if     ( wParam==joyKeyVars[i][JOY_UP] )    joy[i][2] = 0;
                else if( wParam==joyKeyVars[i][JOY_DOWN] )  joy[i][2] = 0;
                else if( wParam==joyKeyVars[i][JOY_LEFT] )  joy[i][1] = 0;
                else if( wParam==joyKeyVars[i][JOY_RIGHT] ) joy[i][1] = 0;
                else if( wParam==joyKeyVars[i][JOY_B1] )    joy[i][0] |= 1;
                else if( wParam==joyKeyVars[i][JOY_B2] )    joy[i][0] |= 2;
                else if( wParam==joyKeyVars[i][JOY_B3] )    joy[i][0] |= 4;
                else if( wParam==joyKeyVars[i][JOY_B4] )    joy[i][0] |= 8;
                else joykey = 0;
              }
            
            if( joykey )
              {
                dazzler_send(hwnd, ((joykey==1)^g_joy_swap) ? g_joy1 : g_joy2, 3);
                if (g_joy_show) set_window_title(hwnd);
              }
            else if( wParam==' ' ) 
              {
                // space bar is button 1 for both joysticks
                g_joy1[0] |= 1;
                g_joy2[0] |= 1;
                dazzler_send(hwnd, g_joy1, 3);
                dazzler_send(hwnd, g_joy2, 3);
                if( g_joy_show ) set_window_title(hwnd);
              }
          }

        break;
      }

    case WM_LBUTTONDBLCLK:
      {
        toggle_fullscreen(hwnd);
        break;
      }

    case WM_TIMER:
      {
        set_window_title(hwnd);
        SetTimer(hwnd, 0, 2000, NULL);
        break;
      }

    case WM_DESTROY:
      PostQuitMessage(0);
      break;

    case WM_GETMINMAXINFO:
      {
        LPMINMAXINFO lpMMI = (LPMINMAXINFO)lParam;
        lpMMI->ptMinTrackSize.x = 128;
        lpMMI->ptMinTrackSize.y = 128+20;
        calc_window_size(&lpMMI->ptMinTrackSize.x, &lpMMI->ptMinTrackSize.y);
        break;
      }

    case WM_CREATE:
      joySetCapture(hwnd, JOYSTICKID1, NULL, true);
      joySetCapture(hwnd, JOYSTICKID2, NULL, true);
      break;

    case MM_JOY1MOVE:
    case MM_JOY1BUTTONUP:
    case MM_JOY1BUTTONDOWN:
    case MM_JOY2MOVE:
    case MM_JOY2BUTTONUP:
    case MM_JOY2BUTTONDOWN:
      {
        byte msg[3];

        if( (uMsg==MM_JOY1MOVE || uMsg==MM_JOY1BUTTONUP || uMsg==MM_JOY1BUTTONDOWN) ^ g_joy_swap )
          msg[0] = DAZ_JOY1;
        else
          msg[0] = DAZ_JOY2;

        if( !(wParam & JOY_BUTTON1) ) msg[0] |= 1;
        if( !(wParam & JOY_BUTTON2) ) msg[0] |= 2;
        if( !(wParam & JOY_BUTTON3) ) msg[0] |= 4;
        if( !(wParam & JOY_BUTTON4) ) msg[0] |= 8;

        msg[1] = get_joy_value(LOWORD(lParam));
        msg[2] = get_joy_value(0xFFFF-HIWORD(lParam));

        dazzler_send(hwnd, msg, 3);
        memcpy((msg[0] & 0xF0)==DAZ_JOY1 ? g_joy1 : g_joy2, msg, 3);
        if( g_joy_show ) set_window_title(hwnd);
        break;
      }

    case ID_SOCKET:
      {
        byte data[2500];
        int size = recv(server_socket, (char *) data, 2500, 0);
        if( size>0 ) 
          {
            static bool skip_greeting = true;
            if( skip_greeting )
              {
                // when connecting, PC host sends a greeting message saying
                // "[connected as nth client on port 8800]"
                // we need to ignore that
                int i;
                for(i=0; i<size && skip_greeting; i++) 
                  if( data[i]=='\n' )
                    skip_greeting = false; 
                
                if( i<size ) dazzler_receive(hwnd, data+i, size-i);
              }
            else
              dazzler_receive(hwnd, data, size);
          }
        else
          {
            server_socket = INVALID_SOCKET;
            set_window_title(hwnd);
          }
        break;
      }
      
    default:
      return DefWindowProc(hwnd, uMsg, wParam, lParam);
    }
    
  return 0;
}


int WINAPI wWinMain(HINSTANCE hInstance, HINSTANCE, PWSTR pCmdLine, int nCmdShow)
{
  int p = -1, baud = 1050000;
  read_settings(&p, &baud);
		
  // Register the main window class.
  const wchar_t CLASS_NAME[]  = L"Dazzler Window Class";
  WNDCLASS wc = { };
  wc.lpfnWndProc   = WindowProc;
  wc.hInstance     = hInstance;
  wc.lpszClassName = CLASS_NAME;
  wc.style         = CS_DBLCLKS;
  wc.hCursor       = LoadCursor(NULL, IDC_ARROW);
  RegisterClass(&wc);

  // Create the main window.
  long w = 128*4, h = 128*4;
  calc_window_size(&w, &h);
  HWND hwnd = CreateWindowEx(0, CLASS_NAME, L"Dazzler Display", WS_OVERLAPPEDWINDOW,
                             CW_USEDEFAULT, CW_USEDEFAULT, w, h,
                             NULL, NULL, hInstance, NULL);

  // if we couldn't create the window then just exit
  if( hwnd == NULL ) return 0;

  // create the window menu
  HMENU menu = CreateMenu();
  HMENU menuFile = CreateMenu();
  AppendMenu(menuFile, MF_BYPOSITION | MF_STRING, ID_FILE_EXIT, L"E&xit");
  HMENU menuAspect = CreateMenu();
  AppendMenu(menuAspect, MF_BYPOSITION | MF_STRING, ID_VIEW_ASPECT_11, L"&1:1");
  AppendMenu(menuAspect, MF_BYPOSITION | MF_STRING, ID_VIEW_ASPECT_43, L"&4:3");
  AppendMenu(menuAspect, MF_BYPOSITION | MF_STRING, ID_VIEW_ASPECT_WIN, L"&Stretch");
  HMENU menuView = CreateMenu();
  AppendMenu(menuView, MF_BYPOSITION | MF_STRING, ID_VIEW_FULLSCREEN, L"&Full Screen\tCtrl+F");
  AppendMenu(menuView, MF_BYPOSITION | MF_STRING, ID_VIEW_NORMAL,     L"&Normal\tCtrl+N");
  AppendMenu(menuView, MF_POPUP, (UINT_PTR) menuAspect, L"&Pixel Aspect Ratio");
  HMENU menuPort = CreateMenu();
  AppendMenu(menuPort, MF_BYPOSITION | MF_STRING, ID_SETTINGS_PORT_NONE, L"None");
  HMENU menuBaud = CreateMenu();
  AppendMenu(menuBaud, MF_BYPOSITION | MF_STRING, ID_SETTINGS_BAUD_9600, L"9600");
  AppendMenu(menuBaud, MF_BYPOSITION | MF_STRING, ID_SETTINGS_BAUD_38400, L"38400");
  AppendMenu(menuBaud, MF_BYPOSITION | MF_STRING, ID_SETTINGS_BAUD_115200, L"115200");
  AppendMenu(menuBaud, MF_BYPOSITION | MF_STRING, ID_SETTINGS_BAUD_250000, L"250000");
  AppendMenu(menuBaud, MF_BYPOSITION | MF_STRING, ID_SETTINGS_BAUD_525000, L"525000");
  AppendMenu(menuBaud, MF_BYPOSITION | MF_STRING, ID_SETTINGS_BAUD_750000, L"750000");
  AppendMenu(menuBaud, MF_BYPOSITION | MF_STRING, ID_SETTINGS_BAUD_1050000, L"1050000");
  HMENU menuSettings = CreateMenu();
  AppendMenu(menuSettings, MF_POPUP, (UINT_PTR) menuPort, L"&Port");
  AppendMenu(menuSettings, MF_POPUP, (UINT_PTR) menuBaud, L"&Baud Rate");
  AppendMenu(menuSettings, MF_BYPOSITION | MF_STRING, ID_SETTINGS_JOY_SWAP, L"Swap &Joysticks\tCtrl+J");
  AppendMenu(menuSettings, MF_BYPOSITION | MF_STRING, ID_SETTINGS_JOY_SHOW, L"&Show Joysticks");
  AppendMenu(menuSettings, MF_BYPOSITION | MF_STRING, ID_SETTINGS_JOY_KEYS, L"&Keyboard Joysticks");
  AppendMenu(menuSettings, MF_BYPOSITION | MF_STRING, ID_SETTINGS_AUDIO_MUTE, L"Mute &Audio");
  HMENU menuHelp = CreateMenu();
  AppendMenu(menuHelp, MF_BYPOSITION | MF_STRING, ID_HELP_ABOUT, L"&About");

  AppendMenu(menu, MF_POPUP, (UINT_PTR) menuFile, L"&File");
  AppendMenu(menu, MF_POPUP, (UINT_PTR) menuView, L"&View");
  AppendMenu(menu, MF_POPUP, (UINT_PTR) menuSettings, L"&Settings");
  AppendMenu(menu, MF_POPUP, (UINT_PTR) menuHelp, L"&Help");
  SetMenu(hwnd, menu);

  // check the serial port and baud rate settings
  find_com_ports(hwnd);     
  if( wcsncmp(pCmdLine, L"COM", 3)==0 && wcslen(pCmdLine)<7 )
    p = _wtoi(pCmdLine+3);
  else if (wcslen(pCmdLine) > 0) 
	p = -1;

  // if we found a valid COM port then set it
  if( p>0 && p<256 ) set_com_port(hwnd, p);
  set_baud_rate(hwnd, baud);

  // set initial menu settings
  CheckMenuItem(GetSubMenu(GetMenu(hwnd), 2), ID_SETTINGS_JOY_SWAP, MF_BYCOMMAND | (g_joy_swap ? MF_CHECKED : MF_UNCHECKED));
  CheckMenuItem(GetSubMenu(GetMenu(hwnd), 2), ID_SETTINGS_JOY_SHOW, MF_BYCOMMAND | (g_joy_show ? MF_CHECKED : MF_UNCHECKED));
  CheckMenuItem(GetSubMenu(GetMenu(hwnd), 2), ID_SETTINGS_JOY_KEYS, MF_BYCOMMAND | (g_joy_keys ? MF_CHECKED : MF_UNCHECKED));
  CheckMenuItem(GetSubMenu(GetMenu(hwnd), 2), ID_SETTINGS_AUDIO_MUTE, MF_BYCOMMAND | (g_audio_mute ? MF_CHECKED : MF_UNCHECKED));
  CheckMenuRadioItem(menuAspect, ID_VIEW_ASPECT_11, ID_VIEW_ASPECT_WIN, ID_VIEW_ASPECT_11+g_aspect_ratio, MF_BYCOMMAND);

  // initialize joystick and main memory data
  g_joy1[0]=DAZ_JOY1 | 0x0f; g_joy1[1]=0x00; g_joy1[2]=0x00;
  g_joy2[0]=DAZ_JOY2 | 0x0f; g_joy2[1]=0x00; g_joy2[2]=0x00;
  memset(dazzler_mem, 0, 2*2048);

  if( g_com_port>0 || wcslen(pCmdLine)==0 )
    {
      // start the serial communication thread
      HANDLE h = CreateThread(0, 0, serial_thread, hwnd, 0, NULL);
      CloseHandle(h);
    }
  else
    {
      // no COM port given AND we have a command line argument
      // => take the command line argument as a TCP port to connect to
      peer = pCmdLine;
      server_socket = connect_socket(hwnd, peer);
      if( server_socket==INVALID_SOCKET )
        return 0;
      else if( WSAAsyncSelect(server_socket, hwnd, ID_SOCKET, FD_READ)!=0 )
        return 0;
      set_window_title(hwnd);

      // remove "port" and "baud rate" menu items (not needed in this mode)
      DeleteMenu(menuSettings, 1, MF_BYPOSITION);
      DeleteMenu(menuSettings, 0, MF_BYPOSITION);
    }

  // start the video render thread
  video_start(hwnd);
  adjust_render_area_size(hwnd);
  
  // show the main window
  ShowWindow(hwnd, SW_SHOW);

  // start the audio render thread
  if( !g_audio_mute ) audio_start();

  // start timer to show fps
  SetTimer(hwnd, 0, 1000, NULL);

  // Run the message loop.
  MSG msg = { };
  while( GetMessage(&msg, NULL, 0, 0) )
    {
      TranslateMessage(&msg);
      DispatchMessage(&msg);
    }

  if( server_socket!=INVALID_SOCKET )
    {
      shutdown(server_socket, SD_SEND);
      closesocket(server_socket);
    }

  audio_stop();

  return 0;
}

#endif
