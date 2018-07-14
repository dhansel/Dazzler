// -----------------------------------------------------------------------------
// Cromemco Dazzler emulation for PIC32MX device
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


#include "app.h"

#include "peripheral/oc/plib_oc.h"
#include "peripheral/tmr/plib_tmr.h"
#include "peripheral/int/plib_int.h"
#include "peripheral/adc/plib_adc.h"
#include "peripheral/usart/plib_usart.h"


// If 1, talk to Arduino Due Native port via USB. Pins 21 and 22 are USB D+/D- pins
// If 0, talk to a 3.3v serial connection 750000 baud 8N1. Pins 21 and 22 are TX and RX
#define USE_USB 1

// 0=do not produce any signals unless Dazzler control register is set to "on"
// 1=produce sync signals but black screen if Dazzler control register is off
#define ALWAYS_ON 1


// The following Microchip USB host stack source files have been modified from their original:
//
// File: firmware\src\system_config\default\framework\usb\src\dynamic\usb_host_cdc.c
// line 790: changed USB_CDC_PROTOCOL_AT_V250 to USB_CDC_PROTOCOL_NO_CLASS_SPECIFIC
// (Arduino Due reports CDC protocol as NO_CLASS_SPECIFIC)
//
// File: firmware\src\system_config\default\framework\usb\src\dynamic\usb_host.c
// added line 5391: if( transferType==USB_TRANSFER_TYPE_BULK && maxPacketSize>64 ) maxPacketSize=64;
//(Arduino Due always sends the high speed descriptor when configuration is requested, should
// send full speed descriptor if the host is full speed. Will send full speed descriptor when OTHER 
// configuration is requested. maxPacketSize for full speed bulk transfer is 64 bytes, high speed is 512)
// 
// File: firmware\src\system_config\default\framework\driver\usb\usbfs\src\dynamic\drv_usbfs_host.c
// added starting in line 1713:
// if( ++pipe->nakCounter>=10 )
//   { pIRP->status = USB_HOST_IRP_STATUS_COMPLETED_SHORT; pipe->nakCounter = 0; endIRP = true; }
//  else
//  (to end a read request if no data is received after waiting for 10ms, necessary
//   so we can still send joystick data when the Arduino is not sending)


// VGA picture generation: We produce the timings for a SVGA 800x600 picture.
// However, we only have to display 128x128 actual pixels. Vertically, lines
// are scaled up by factor 4 for a total of 512 visible lines.
// Horizontally, the code for writing out pixel data (in function IntHandlerTimer2)
// outputs pixels at a ~9.7MHz rate. Since the expected pixel clock for this VGA
// mode is 40MHz, each horizontal pixel is also scaled up by about factor 4,
// resulting in (mostly) square pixels. 
// Timings were taken from: http://www.tinyvga.com/vga-timing/800x600@60Hz

// Horizontal pixels are counted by timer2 which runs at 24MHz
// - back porch ends (visible area starts) at 0 (i.e. when timer2 runs
//   over and causes the interrupt that writes pixel data to PORTB)
// - front porch starts when interrupt routine is finished writing data
// - front porch ends at HSYNC_START
// - back porch starts at HSYNC_START+HSYNC_LENGTH
// Vertical lines are counted by g_current_line, which is incremented
// at the end of the interrupt routine that outputs a line.
// - back porch ends (and visible area starts) at line VBP_LENGTH
// - front porch starts at VBP_LENGTH+DISPLAY_LINES
// - front porch ends at NUM_LINES-VSYNC_LENGTH
// - back porch starts at NUM_LINES (=0)

// All numbers for horizontal timing are cycles of the peripheral clock @24MHz,
// 1 cycle at 24MHz is ~0.0416667us.
// The pixel counts do not line up with the 800x600 spec since per spec the clock
// should be 40MHz and our timer runs at 24MHz. But the actual times do (mostly) 
// match the spec.  HFP_LENGTH is not used - the front porch is just the time
// between when the timer2 interrupt finishes and the next sync pulse.
// Since we use only a portion of horizontally visible area, we split up the 
// remainder (margin) between front and back porch. We don't split evenly since
// the back porch is actually longer than what we specify here due to the time
// it takes between the timer overrun and the interrupt routine actually
// starting to push out pixels. The exact timing values were determined 
// experimentally such that the display appears in the middle of the screen.
#define NUM_PIXELS	   634			   // =26.417us/line (spec: 26.4us)
#define HFP_LENGTH     (24+110)        // (1us + 4.583us) front porch plus margin
#define HBP_LENGTH     (53+50)         // (2.208us + 2.083us(+x)) back porch plus margin
#define HSYNC_LENGTH   77			   // =3.208us  (spec: 3.2us)
#define DISPLAY_PIXELS 320             // =13.3us (128px @ 9.7MHz=13.196us)
#define HSYNC_START	   (NUM_PIXELS-HBP_LENGTH-HSYNC_LENGTH)

// sanity check
#if (HFP_LENGTH+HBP_LENGTH+HSYNC_LENGTH+DISPLAY_PIXELS) != NUM_PIXELS
#error Inconsistent horizontal timing!
#endif


// All numbers for vertical timing are numbers of horizontal lines,
// each horizontal line is 0.026417ms.
// The 800x600 resolution gives us 600 visible lines but we only need 512
// (Dazzler has 128 lines which we scale up by 4). So there are 88 lines
// of margin, which we split between the front and back porches.
#define NUM_LINES     628               // =16.59ms/frame (spec: 16.579ms)
#define VFP_LENGTH    (1+44)            // front porch plus margin
#define VBP_LENGTH    (23+44) 	        // back porch plus margin
#define VSYNC_LENGTH  4                 // =0.10567ms (spec: 0.1056ms)
#define DISPLAY_LINES (128*4)

// sanity check
#if (VFP_LENGTH+VBP_LENGTH+VSYNC_LENGTH+DISPLAY_LINES) != NUM_LINES
#error Inconsistent vertical timing!
#endif


// current scan line
volatile int g_current_line = 0;


// Dazzler control register:
// D7: on/off
// D6-D0: screen memory location (not used in client)
uint8_t dazzler_ctrl = 0x00;


// Dazzler picture control register:
// D7: not used
// D6: 1=resolution x4, 0=normal resolution
// D5: 1=2k memory, 0=512byte memory
// D4: 1=color, 0=monochrome
// D3-D0: color info for x4 high res mode
uint8_t dazzler_picture_ctrl = 0x10;


// frame buffer has 128 lines of 128+1 columns (one extra 0 at the end)
// (use 132 bytes per line to stay 32-bit aligned)
uint8_t framebuffer[128][132] __attribute__((aligned(32)));


// dazzler video memory, necessary so we can update the frame buffer properly 
// if the dazzler_picture_ctrl register is changed and to avoid re-drawing
// the framebuffer for bytes that have not changed
uint8_t dazzler_mem[2048];


// test mode (see function draw_test_screen)
int test_mode = 0;

// dazzler commands received from the Altair simulator
#define DAZ_MEMBYTE   0x10
#define DAZ_FULLFRAME 0x20
#define DAZ_CTRL      0x30
#define DAZ_CTRLPIC   0x40

// dazzler commands sent to the Altair simulator
#define DAZ_JOY1      0x10
#define DAZ_JOY2      0x20

// receiver states
#define ST_IDLE      0
#define ST_MEMBYTE1  1
#define ST_MEMBYTE2  2
#define ST_CTRL      3
#define ST_CTRLPIC   4
#define ST_FULLFRAME 5


// -----------------------------------------------------------------------------
// --------------- display routines (updating the frame buffer) ----------------
// -----------------------------------------------------------------------------


void update_byte_bigmem_single(int i, uint8_t b)
{
  // 2K RAM, high resolution (128x128 pixels, common color)
  int x, y;
  uint8_t color, *fb;

  // determine position within quadrant
  x = (i & 0x000f)*4;
  y = (i & 0x01f0)/8;

  // determine quadrant
  if( i & 0x0200 ) x += 64;
  if( i & 0x0400 ) y += 64; 

  fb    = &(framebuffer[y][x]);
  color = dazzler_picture_ctrl & 0x0f;

  fb[0] = b & 0x01 ? color : 0;
  fb[1] = b & 0x02 ? color : 0;
  fb[2] = b & 0x10 ? color : 0;
  fb[3] = b & 0x20 ? color : 0;
  fb += sizeof(framebuffer[0]); // next line
  fb[0] = b & 0x04 ? color : 0;
  fb[1] = b & 0x08 ? color : 0;
  fb[2] = b & 0x40 ? color : 0;
  fb[3] = b & 0x80 ? color : 0;
}


void update_byte_bigmem_multi(int i, uint8_t b)
{
  // 2K RAM, low resolution (64x64 pixels, individual color)
  int x, y;
  uint8_t color1, color2, *fb; 

  // determine position within quadrant
  x = (i & 0x000f)*4;
  y = (i & 0x01f0)/8;

  // determine quadrant
  if( i & 0x0200 ) x += 64;
  if( i & 0x0400 ) y += 64; 

  fb     = &(framebuffer[y][x]);
  color1 = b & 0x0f;
  color2 = (b & 0xf0)/16;

  fb[0] = color1; fb[1] = color1;
  fb[2] = color2; fb[3] = color2;
  fb += sizeof(framebuffer[0]); // next line
  fb[0] = color1; fb[1] = color1;
  fb[2] = color2; fb[3] = color2;
}


void update_byte_smallmem_single(int i, uint8_t b)
{
  // 512 bytes RAM, high resolution (64x64 pixels, common color)
  int x, y;
  uint8_t color, *fb;
  uint8_t color0, color1, color2, color3, color4, color5, color6, color7;

  // determine position
  x = (i & 0x000f)*8;
  y = (i & 0x01f0)/4;

  fb     = &(framebuffer[y][x]);
  color  = dazzler_picture_ctrl & 0x0f;
  color0 = b & 0x01 ? color : 0;
  color1 = b & 0x02 ? color : 0;
  color2 = b & 0x04 ? color : 0;
  color3 = b & 0x08 ? color : 0;
  color4 = b & 0x10 ? color : 0;
  color5 = b & 0x20 ? color : 0;
  color6 = b & 0x40 ? color : 0;
  color7 = b & 0x80 ? color : 0;

  fb[0] = color0; fb[1] = color0;
  fb[2] = color1; fb[3] = color1;
  fb[4] = color4; fb[5] = color4;
  fb[6] = color5; fb[7] = color5;
  fb += sizeof(framebuffer[0]); // next line
  fb[0] = color0; fb[1] = color0;
  fb[2] = color1; fb[3] = color1;
  fb[4] = color4; fb[5] = color4;
  fb[6] = color5; fb[7] = color5;
  fb += sizeof(framebuffer[0]); // next line
  fb[0] = color2; fb[1] = color2;
  fb[2] = color3; fb[3] = color3;
  fb[4] = color6; fb[5] = color6;
  fb[6] = color7; fb[7] = color7;
  fb += sizeof(framebuffer[0]); // next line
  fb[0] = color2; fb[1] = color2;
  fb[2] = color3; fb[3] = color3;
  fb[4] = color6; fb[5] = color6;
  fb[6] = color7; fb[7] = color7;
}



void update_byte_smallmem_multi(int i, uint8_t b)
{
  // 512 bytes RAM, low resolution (32x32 pixels, individual color)
  int x, y;
  uint8_t color1, color2, *fb;

  // determine position
  x = (i & 0x000f)*8;
  y = (i & 0x01f0)/4;

  fb     = &(framebuffer[y][x]);
  color1 = b & 0x0f;
  color2 = (b & 0xf0)/16;

  fb[0] = color1; fb[1] = color1;
  fb[2] = color1; fb[3] = color1;
  fb[4] = color2; fb[5] = color2;
  fb[6] = color2; fb[7] = color2;
  fb += sizeof(framebuffer[0]); // next line
  fb[0] = color1; fb[1] = color1;
  fb[2] = color1; fb[3] = color1;
  fb[4] = color2; fb[5] = color2;
  fb[6] = color2; fb[7] = color2;
  fb += sizeof(framebuffer[0]); // next line
  fb[0] = color1; fb[1] = color1;
  fb[2] = color1; fb[3] = color1;
  fb[4] = color2; fb[5] = color2;
  fb[6] = color2; fb[7] = color2;
  fb += sizeof(framebuffer[0]); // next line
  fb[0] = color1; fb[1] = color1;
  fb[2] = color1; fb[3] = color1;
  fb[4] = color2; fb[5] = color2;
  fb[6] = color2; fb[7] = color2;
}


void (*update_byte)(int, uint8_t) = &update_byte_smallmem_multi;

void set_update_byte()
{
  switch( dazzler_picture_ctrl & 0x60 )
    {
    case 0x20 : update_byte = update_byte_bigmem_multi;    break;
    case 0x60 : update_byte = update_byte_bigmem_single;   break;
    case 0x00 : update_byte = update_byte_smallmem_multi;  break;
    case 0x40 : update_byte = update_byte_smallmem_single; break;
    }
}


void dazzler_receive(uint8_t data)
{
  static int state = ST_IDLE, addr, cnt;

  switch( state )
    {
    case ST_IDLE:
      {
        switch( data & 0xf0 )
          {
          case DAZ_MEMBYTE: 
            state = ST_MEMBYTE1;
            addr  = (data & 0x07) * 256;
            break;

          case DAZ_CTRL:
            state = ST_CTRL;
            break;

          case DAZ_CTRLPIC:
            state = ST_CTRLPIC;
            break;

          case DAZ_FULLFRAME:
            state = ST_FULLFRAME;
            addr  = 0;
            cnt   = (data & 0x0f) ? 2048 : 512;
            break;
          }
        break;
      }
      
    case ST_MEMBYTE1:
      addr += data;
      state = ST_MEMBYTE2;
      break;

    case ST_MEMBYTE2:
      if( data != dazzler_mem[addr] ) { update_byte(addr, data); dazzler_mem[addr] = data; }
      state = ST_IDLE;
      break;

    case ST_CTRL:
      dazzler_ctrl = data;
#if ALWAYS_ON==0
      // start/stop generating the output signal
      g_current_line = 0;
      if( dazzler_ctrl & 0x80 )
        PLIB_TMR_Start(TMR_ID_2);
      else
        PLIB_TMR_Stop(TMR_ID_2);
#endif      
      state = ST_IDLE;
      break;

    case ST_CTRLPIC:
      if( data != dazzler_picture_ctrl )
        {
          if( data != dazzler_picture_ctrl )
            {
              uint8_t ctrl = dazzler_picture_ctrl;
              dazzler_picture_ctrl = data;
              set_update_byte();

              if( (data & 0x20)!=(ctrl & 0x20) )
                {
                  // redraw the full frame if memory size setting has changed
                  int i, w = (dazzler_picture_ctrl & 0x20) ? 2048 : 512;
                  for(i=0; i<w; i++) update_byte(i, dazzler_mem[i]);
                }
              else if( (data & 0x40)!=(ctrl & 0x40) || ((ctrl & 0x40) && (data & 0x0f)!=(ctrl & 0x0f)) )
                {
                  // redraw only the foreground if the 1x/4x resolution setting has changed or
                  // we are in 4x resolution mode and the (common) color has changed
                  int i, w = (dazzler_picture_ctrl & 0x20) ? 2048 : 512;
                  for(i=0; i<w; i++)
                    if( dazzler_mem[i]!=0 )
                      update_byte(i, dazzler_mem[i]);
                }

              ColorStateSet((dazzler_picture_ctrl & 0x10)!=0);
            }
        }

      state = ST_IDLE;
      break;

    case ST_FULLFRAME:
      if( data != dazzler_mem[addr] ) { update_byte(addr, data); dazzler_mem[addr] = data;}
      addr++;
      if( --cnt==0 ) state = ST_IDLE; 
      break;
    }
}


// -----------------------------------------------------------------------------
// --------------------------- ring-buffer handling ----------------------------
// -----------------------------------------------------------------------------


uint32_t ringbuffer_start = 0, ringbuffer_end = 0;
uint8_t  ringbuffer[0x1000];

#define ringbuffer_full()     (((ringbuffer_end+1)&0x0fff) == ringbuffer_start)
#define ringbuffer_empty()      (ringbuffer_start==ringbuffer_end)
#define ringbuffer_available() ((ringbuffer_start-ringbuffer_end-1)&0x0fff)


inline void ringbuffer_enqueue(uint8_t b)
{
  // There's really not much we can do if we receive a byte of data
  // when the ring buffer is full. Overwriting the beginning of the buffer
  // is about as bad as dropping the newly received byte. So we save
  // the time to check whether the buffer is full and just overwrite.
  ringbuffer[ringbuffer_end] = b;
  ringbuffer_end = (ringbuffer_end+1) & 0x0fff;
}

inline uint8_t ringbuffer_dequeue()
{
  if( !ringbuffer_empty() )
    {
      dazzler_receive(ringbuffer[ringbuffer_start]);
      ringbuffer_start = (ringbuffer_start+1) & 0x0fff;
    }
}


#if USE_USB==0
inline void ringbuffer_enqueue_usart()
{
  if( PLIB_USART_ReceiverDataIsAvailable(USART_ID_2) )
    ringbuffer_enqueue(PLIB_USART_ReceiverByteReceive(USART_ID_2));
}
#endif


// -----------------------------------------------------------------------------
// ----------------------------- test mode handling ----------------------------
// -----------------------------------------------------------------------------


void draw_joystick_pixel(int x, int y)
{
  static int px, py, pc = -1;
  x = x/2+64;
  y = 63-y/2;
  if( pc>=0 ) framebuffer[py][px] = pc;
  pc = framebuffer[y][x];
  px = x;
  py = y;
  framebuffer[y][x] = 0x0f;
}


void draw_joystick_buttons(uint8_t buttons)
{
  framebuffer[0][61] = (buttons & 0x01) ? 0x04 : 0x02;
  framebuffer[0][63] = (buttons & 0x02) ? 0x04 : 0x02;
  framebuffer[0][65] = (buttons & 0x04) ? 0x04 : 0x02;
  framebuffer[0][67] = (buttons & 0x08) ? 0x04 : 0x02;
}


void draw_test_screen()
{
  int r, c;
  switch( test_mode )
    {
    case 1:
    case 2:
      // joystick test screen
      for(r=0; r<128; r++)
        {
          framebuffer[r][0]   = 0x01;
          framebuffer[r][64]  = (r & 1) && r<127 ? 0x01 : 0x00;
          framebuffer[r][127] = 0x01;
        }
      for(c=0; c<128; c++)
        {
          framebuffer[0][c]   = 0x01;
          framebuffer[63][c]  = (c & 1) && c<127 ? 0x00 : 0x01;
          framebuffer[127][c] = 0x01;
        }
      break;
                        
    case 11:
      // color test pattern
      ColorOn();
      for(r=0; r<32; r++)
        for(c=0; c<16; c++)
          update_byte_smallmem_multi(r*16+c, ((r+c)&7) + 16*(((r+c)&7)+8));
      break;
                        
    case 12:
      // gray-scale test pattern
      ColorOff();
      for(r=0; r<32; r++)
        for(c=0; c<16; c++)
          update_byte_smallmem_multi(r*16+c, ((r+c*2)&15) + 16*((r+c*2+1)&15));
      break;
                      
    case 13:
      // black-and-white grid pattern
      ColorOff();
      for(r=0; r<128; r++)
        for(c=0; c<128; c++)
          framebuffer[r][c] = r&1 ? 0xff : ((c&1) ? 0x00 : 0xff);
      break;
    }
}


void check_test_button()
{
  static int debounce = 0;
    
  if( g_current_line==2 && debounce==0 && !TestButtonStateGet() )
    {
      // button press detected
      debounce = 1;
    }
  else if( g_current_line==1 && debounce==1 && !TestButtonStateGet() )
    {
      // button still pressed after one frame => valid button press (not a bounce))
      test_mode = test_mode+1;
      if( test_mode>13 ) test_mode = 11;
      draw_test_screen();
      debounce = 2;
    }
  else if( TestButtonStateGet() )
  {
    // button no longer pressed
    debounce = 0;
  }
}


// -----------------------------------------------------------------------------
// ----------------------------- joystick handling -----------------------------
// -----------------------------------------------------------------------------

#if USE_USB>0
volatile uint64_t usb_joydata = 0;
#endif


#define AVGC 4
int rolling_average(int n, int v)
{
  static int8_t ptr[4] = {-1,-1,-1,-1};
  static int buf[4][AVGC], sum[4];
  int i, rv;
  
  if( ptr[n]<0 )
  {
      for(i=0; i<AVGC; i++) buf[n][i] = v;
      sum[n]=v*AVGC;
      ptr[n]=0;
  }
  else
  {
      sum[n] += v - buf[n][ptr[n]];
      buf[n][ptr[n]] = v;
      ptr[n]++;
      if( ptr[n]>=AVGC ) ptr[n]=0;
  }
  
  return sum[n]/AVGC;
}


inline char scale_joystick_pot(int v, int center)
{
  v = center-v;
  
  if( v<0 && center!=1023 )
    v = v*128/(1023-center);
  else if( v>0 && center!=0 )
    v = v*128/center;
    
  // Some games can have problems if the joystick values 
  // go all the way to extremes -128/127. For example there seems to be
  // a bug in Gotcha where if the initial value read for the joystick 
  // is 0 then a full left or full down (value -128) will actually move
  // the player in the opposite direction.
  // So we keep the values in range -127..126 to avoid that.
  if( v>=-4 && v<=4 ) 
   return 0; 
  else if( v<-127 )
    return -127;
  else if( v>126 )
    return 126;
  else
    return v;
}


uint8_t read_joystick_buttons()
{
  int delay;
  uint8_t i, b = 0;

  // turn off clock
  ButtonsClockOff();
   
  // produce low pulse on "shift/load"
  ButtonsShiftOff();
  for(delay=0; delay<5; delay++) asm volatile("nop");
  ButtonsShiftOn();

  for(i=0; i<8; i++)
    {
      // read data
      b = b * 2;
      if( ButtonsDataStateGet() ) b |= 1;

      // pulse clock
      ButtonsClockOn();
      for(delay=0; delay<5; delay++) asm volatile("nop");
      ButtonsClockOff();
    }
   
  ButtonsShiftOff();
  return b;
}


void check_joystick()
{
  int v;
  static int joy1_center_x = -1, joy1_center_y = -1;
  static int joy2_center_x = -1, joy2_center_y = -1;
  static int x1  = 0, y1  = 0, x2  = 0, y2  = 0, b1  = 0, b2  = 0;
  static int x1p = 0, y1p = 0, x2p = 0, y2p = 0, b1p = 0, b2p = 0;
  static int m = 0;

  // during the first few lines of the vertical back porch
  // do one "joystick action" per scan line
  if( g_current_line==m )
    {
      switch( m )
        {
        case 0: 
          // set ADC to read AN9 input (joystick 1, x axis)
          PLIB_ADC_MuxChannel0InputPositiveSelect(ADC_ID_1, ADC_MUX_A, ADC_INPUT_POSITIVE_AN9);
          PLIB_ADC_SamplingStart(ADC_ID_1);
          m++;
          break;

        case 1: 
          // read joystick 1, x axis, initialize center on first read
          v = rolling_average(0, PLIB_ADC_ResultGetByIndex(ADC_ID_1, 0));
          if( joy1_center_x < 0 ) joy1_center_x = v;
          x1 = scale_joystick_pot(v, joy1_center_x);
          // set ADC to read AN10 input (joystick 1, y axis)
          PLIB_ADC_MuxChannel0InputPositiveSelect(ADC_ID_1, ADC_MUX_A, ADC_INPUT_POSITIVE_AN10);
          PLIB_ADC_SamplingStart(ADC_ID_1);
          m++;
          break;
                   
        case 2: 
          // read joystick 1, y axis, initialize center on first read
          v = rolling_average(1, PLIB_ADC_ResultGetByIndex(ADC_ID_1, 0));
          if( joy1_center_y < 0 ) joy1_center_y = v;
          y1 = scale_joystick_pot(v, joy1_center_y);
          // set ADC to read AN0 input (joystick 2, x axis)
          PLIB_ADC_MuxChannel0InputPositiveSelect(ADC_ID_1, ADC_MUX_A, ADC_INPUT_POSITIVE_AN0);
          PLIB_ADC_SamplingStart(ADC_ID_1);
          m++;
          break;
                   
        case 3: 
          // read joystick 2, x axis, initialize center on first read
          v = rolling_average(2, PLIB_ADC_ResultGetByIndex(ADC_ID_1, 0));
          if( joy2_center_x < 0 ) joy2_center_x = v;
          x2 = scale_joystick_pot(v, joy2_center_x);
          // set ADC to read AN1 input (joystick 2, y axis)
          PLIB_ADC_MuxChannel0InputPositiveSelect(ADC_ID_1, ADC_MUX_A, ADC_INPUT_POSITIVE_AN1);
          PLIB_ADC_SamplingStart(ADC_ID_1);
          m++;
          break;
                   
        case 4: 
          // read joystick 2, y axis, initialize center on first read
          v = rolling_average(3, PLIB_ADC_ResultGetByIndex(ADC_ID_1, 0));
          if( joy2_center_y < 0 ) joy2_center_y = v;
          y2 = scale_joystick_pot(v, joy2_center_y);
          m++;
          break;
                   
        case 5: 
          {
            // read joystick buttons
            uint8_t b = read_joystick_buttons();
            b1 = b & 0x0f;
            b2 = b / 16;
            m++;
            break;
          }
              
        case 6: 
          {
#if USE_USB>0
            // copy joystick 1 data to USB send buffer
            uint8_t *jd = (uint8_t *) &usb_joydata;
            jd[0] = DAZ_JOY1 | b1;
            jd[1] = x1;
            jd[2] = y1;
#else
            // check if there are any any changes for joystick 1
            if( x1!=x1p || y1!=y1p || b1 != b1p )
              {
                // send joystick 1 data over serial
                PLIB_USART_TransmitterByteSend(USART_ID_2, DAZ_JOY1 | b1);
                PLIB_USART_TransmitterByteSend(USART_ID_2, x1);
                PLIB_USART_TransmitterByteSend(USART_ID_2, y1);
             
                // remember current values
                x1p = x1; y1p = y1; b1p = b1;
              }
#endif
            // draw calibration info if in calibration mode
            if( test_mode==1 ) 
              {
                draw_joystick_pixel(x1, y1);
                draw_joystick_buttons(b1);
              }
 
            m++;
            break;
          }
          
        case 7:
          {
#if USE_USB>0
            // copy joystick 2 data to USB send buffer
            uint8_t *jd = (uint8_t *) &usb_joydata;
            jd[3] = DAZ_JOY2 | b2;
            jd[4] = x2;
            jd[5] = y2;
#else          
            // check if there are any any changes for joystick 2
            if( x2!=x2p || y2!=y2p || b2 != b2p )
              {
                // send joystick 2 data over serial
                PLIB_USART_TransmitterByteSend(USART_ID_2, DAZ_JOY2 | b2);
                PLIB_USART_TransmitterByteSend(USART_ID_2, x2);
                PLIB_USART_TransmitterByteSend(USART_ID_2, y2);

                // remember current values
                x2p = x2; y2p = y2; b2p = b2;
              }
#endif

            // draw calibration info if in calibration mode
            if( test_mode==2 ) 
              {
                draw_joystick_pixel(x2, y2);
                draw_joystick_buttons(b2);
              }
          
            m=0;
            break;
          }
        }
    }
}


// -----------------------------------------------------------------------------
// -------------------------------- video output -------------------------------
// -----------------------------------------------------------------------------


void __ISR(_TIMER_2_VECTOR, ipl7AUTO) IntHandlerTimer2(void)
{
  // This interrupt happens when timer2 runs over, which signifies
  // the end of the horizontal back porch (i.e. left margin).
  // We use this interrupt to time the beginning of picture data
  // and to set up the output compare registers producing the VSYNC pulse

  if( g_current_line>=VBP_LENGTH && g_current_line<(VBP_LENGTH+DISPLAY_LINES) )
    {
      // we are in the vertically visible region
      uint8_t *ptr = framebuffer[(g_current_line-VBP_LENGTH)/4];
      uint8_t *end = ptr + 129;

      if( dazzler_ctrl & 0x80 )
        // The assembly code below outputs the pixels at roughly a
        // 9.7MHz rate (pixel clock). The loop is just the same as the
        // following C code: while( ptr!=end ) LATB = *ptr++;
        // It is in assembly here since it its timing is essential and we
        // do not want compiler optimization settings to influence the code.
        // One small difference is that the assembly code (unlike the
        // C code) only updates the lower 8 bits of LATB, leaving the 
        // upper 24 bits unchanged. That allows us to still use RB8-15
        // as other outputs that do not constantly get overwritten.
        // The ".set noreorder" prevents the assembler from trying to
        // reorganize the code for better performance. Note that the
        // final ADDIU sits in the BNE's "branch delay slot" and gets
        // executed even though it appears to be outside the loop.
        asm volatile ("    .set noreorder    \n"
                      "    ADDIU  %0, %0, 1  \n"      
                      "lp: LBU    $3, -1(%0) \n"
                      "    SB     $3,  0(%2) \n"
                      "    BNE    %0, %1, lp \n"
                      "    ADDIU  %0, %0, 1  \n"
                      :: "d"(ptr), "d"(end), "d"(&LATB) : "$3" );
    }
  else if( g_current_line==NUM_LINES-VSYNC_LENGTH-1 )
    {
      // We are one line before the sync signal (i.e. the end of the vertical 
      // front porch). Set up OC3 (output compare 3) to set 
      // the vertical sync signal high at the next timer2 interrupt
      PLIB_OC_ModeSelect(OC_ID_3, OC_SET_HIGH_SINGLE_PULSE_MODE);
      PLIB_OC_Enable(OC_ID_3);
    }
  else if( g_current_line==NUM_LINES-1 )
    {
      // We are one line before the end of the sync signal (i.e. the beginning
      // of the vertical back porch). Set up OC3 (output compare 3) 
      // to set the vertical sync signal low at the next timer2 interrupt
      PLIB_OC_ModeSelect(OC_ID_3, OC_SET_LOW_SINGLE_PULSE_MODE);
      PLIB_OC_Enable(OC_ID_3);
    }

#if USE_USB==0
  // The line rate is 37300Hz, i.e. each line takes 0.027 milliseconds.
  // At 750000 baud, each character (10 bits) takes 0.013 milliseconds
  // => no more than 3 characters can be received per line
  ringbuffer_enqueue_usart(); 
  ringbuffer_enqueue_usart(); 
  ringbuffer_enqueue_usart();
#endif
    
  // increase line counter and roll over when we reach the bottom of the screen
  if( ++g_current_line==NUM_LINES ) g_current_line=0;

  // allow next interrupt
  PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_2);
}


void __ISR(_OUTPUT_COMPARE_2_VECTOR, ipl6AUTO) IntHandlerOC2(void)
{
  // This interrupt occurs a few cycles before the timer2 interrupt
  // and its purpose is to put the CPU in a defined state that allows
  // the timer2 interrupt to occur precisely at its scheduled time and
  // not be delayed by a few cycles due to the current CPU activity
  // (which causes a "wobbly" picture).
    
#if USE_USB==0
  asm volatile("wait");
#else
  // For some reason putting the CPU into idle mode prevents USB from 
  // working properly (even if PLIB_USB_StopInIdleDisable was called).
  // So instead we just execute enough NOPs to delay until
  // the timer2 interrupt happens.
  asm volatile("\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n");
  asm volatile("\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n");
  asm volatile("\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n\tnop\n");
#endif
  PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_OUTPUT_COMPARE_2);
}

    
// -----------------------------------------------------------------------------
// -------------------------------- USB handlers -------------------------------
// -----------------------------------------------------------------------------

#if USE_USB>0

static USB_HOST_CDC_OBJ    usbCdcObject     = NULL;
static USB_HOST_CDC_HANDLE usbCdcHostHandle = USB_HOST_CDC_HANDLE_INVALID;
static uint8_t usbInData[64];
volatile bool usbBusy = false;


void usbScheduleTransfer()
{
  static uint64_t joydata_prev = 0;

  if( usbBusy )
    {
      // a USB read or write request is currently in process so we can't
      // schedule a transfer right now
      return;
    }
  else if( usb_joydata!=joydata_prev )
    {
      // note that the joystick data is only updated once per frame
      // so at 60fps it can not change fast enough to starve out the reads
      joydata_prev = usb_joydata;
      USB_HOST_CDC_Write(usbCdcHostHandle, NULL, (void *) &joydata_prev, 6);
      usbBusy = true;
    }
  else
    {
      // If there is space available in the ringbuffer then ask the client
      // to send more data. If there is no space then do not start another
      // request until we have processed some data and space is available.
      size_t avail = ringbuffer_available();
      if( avail>0 ) 
        {
          USB_HOST_CDC_Read(usbCdcHostHandle, NULL, usbInData, avail > 64 ? 64 : avail);
          usbBusy = true;
        }
    }
}


void USBHostCDCAttachEventListener(USB_HOST_CDC_OBJ cdcObj, uintptr_t context)
{
  // a client has been attached
  usbCdcObject = cdcObj;
}


USB_HOST_CDC_EVENT_RESPONSE USBHostCDCEventHandler(USB_HOST_CDC_HANDLE cdcHandle, USB_HOST_CDC_EVENT event, void * eventData, uintptr_t context)
{
  USB_HOST_CDC_EVENT_WRITE_COMPLETE_DATA * writeCompleteEventData;
  USB_HOST_CDC_EVENT_READ_COMPLETE_DATA * readCompleteEventData;
    
  switch(event)
    {
    case USB_HOST_CDC_EVENT_READ_COMPLETE:
      {
        readCompleteEventData = (USB_HOST_CDC_EVENT_READ_COMPLETE_DATA *)(eventData);
        if( readCompleteEventData->result == USB_HOST_CDC_RESULT_SUCCESS )
          {
            // received data from the client => put it in the ringbuffer so it can
            // be processed when we get to it
            size_t i;
            for(i=0; i<readCompleteEventData->length; i++)
              ringbuffer_enqueue(usbInData[i]);
          }

        // transfer is finished => schedule the next transfer
        usbBusy = false;
        usbScheduleTransfer();
        break;
      }
        
    case USB_HOST_CDC_EVENT_WRITE_COMPLETE:
      {   
        // transfer is finished => schedule the next transfer
        usbBusy = false;
        usbScheduleTransfer();
        break;
      }
            
    case USB_HOST_CDC_EVENT_DEVICE_DETACHED:
      {
        // USB_HOST_CDC_Close(usbCdcHostHandle);
        usbCdcObject = NULL;
        usbCdcHostHandle = USB_HOST_CDC_HANDLE_INVALID;
        break;
      }
    }
    
  return(USB_HOST_CDC_EVENT_RESPONE_NONE);
}


void usbTasks()
{
  if( usbCdcHostHandle==USB_HOST_CDC_HANDLE_INVALID )
    {
      if( usbCdcObject!=NULL )
        {
          // a USB device was newly attached - try to open it
          usbCdcHostHandle = USB_HOST_CDC_Open(usbCdcObject);
          if(usbCdcHostHandle != USB_HOST_CDC_HANDLE_INVALID)
            {
              // succeeded opening the device => all further processing is in event handler
              USB_HOST_CDC_EventHandlerSet(usbCdcHostHandle, USBHostCDCEventHandler, (uintptr_t)0);
          
              // request data
              USB_HOST_CDC_Read(usbCdcHostHandle, NULL, usbInData, 64);
            }
        }
    }
  else
    {
      // we have a connection => schedule a new transfer if none is currently going
      // (can't allow USB interrupts while scheduling a new transfer)
      asm volatile ("di");
      usbScheduleTransfer();
      asm volatile ("ei");
    }
}
#endif

// -----------------------------------------------------------------------------
// ------------------------------- initialization ------------------------------
// -----------------------------------------------------------------------------


void APP_Initialize ( void )
{
  int r, c, q;

  // set CPU to switch into IDLE mode when executing "wait" instruction
  SYS_DEVCON_SystemUnlock();
  PLIB_OSC_OnWaitActionSet(OSC_ID_0, OSC_ON_WAIT_IDLE);
  SYS_DEVCON_SystemLock();

  // set up timer 2 (at 24MHz)
  PLIB_TMR_ClockSourceSelect(TMR_ID_2, TMR_CLOCK_SOURCE_PERIPHERAL_CLOCK );
  PLIB_TMR_PrescaleSelect(TMR_ID_2, TMR_PRESCALE_VALUE_1);
  PLIB_TMR_Mode16BitEnable(TMR_ID_2);
  PLIB_TMR_Counter16BitClear(TMR_ID_2);
  PLIB_TMR_Period16BitSet(TMR_ID_2, NUM_PIXELS);

  // set up timer 2 interrupt
  PLIB_INT_MultiVectorSelect( INT_ID_0 );
  PLIB_INT_VectorPrioritySet(INT_ID_0, INT_VECTOR_T2, INT_PRIORITY_LEVEL7);
  PLIB_INT_VectorSubPrioritySet(INT_ID_0, INT_VECTOR_T2, INT_SUBPRIORITY_LEVEL0);
  PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_TIMER_2);
  PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_TIMER_2);
    
  // set up output compare for HSYNC signal
  PLIB_OC_ModeSelect(OC_ID_1, OC_DUAL_COMPARE_CONTINUOUS_PULSE_MODE);
  PLIB_OC_BufferSizeSelect(OC_ID_1, OC_BUFFER_SIZE_16BIT);
  PLIB_OC_TimerSelect(OC_ID_1, OC_TIMER_16BIT_TMR2);
  PLIB_OC_Buffer16BitSet(OC_ID_1, HSYNC_START);  // turn on at timer value
  PLIB_OC_PulseWidth16BitSet(OC_ID_1, HSYNC_START+HSYNC_LENGTH); // turn off at timer value
  PLIB_OC_Enable(OC_ID_1);
    
  // set up output compare + interrupt for going into idle mode
  // (see comment in ISR function IntHandlerOC2)
  PLIB_INT_VectorPrioritySet(INT_ID_0, INT_VECTOR_OC2, INT_PRIORITY_LEVEL6);
  PLIB_INT_VectorSubPrioritySet(INT_ID_0, INT_VECTOR_OC2, INT_SUBPRIORITY_LEVEL0);
  PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_OUTPUT_COMPARE_2);
  PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_OUTPUT_COMPARE_2);
  PLIB_OC_ModeSelect(OC_ID_2, OC_TOGGLE_CONTINUOUS_PULSE_MODE);
  PLIB_OC_BufferSizeSelect(OC_ID_2, OC_BUFFER_SIZE_16BIT);
  PLIB_OC_TimerSelect(OC_ID_2, OC_TIMER_16BIT_TMR2);
  PLIB_OC_Buffer16BitSet(OC_ID_2, NUM_PIXELS-20);
  PLIB_OC_Enable(OC_ID_2);
    
  // set up output compare for VSYNC signal
  PLIB_OC_BufferSizeSelect(OC_ID_3, OC_BUFFER_SIZE_16BIT);
  PLIB_OC_TimerSelect(OC_ID_3, OC_TIMER_16BIT_TMR2);
  PLIB_OC_Buffer16BitSet(OC_ID_3, 0);

  // initialize update_byte function
  set_update_byte();
 
  // set up ADC for joystick input
  PLIB_ADC_ConversionTriggerSourceSelect(ADC_ID_1, ADC_CONVERSION_TRIGGER_INTERNAL_COUNT);
  PLIB_ADC_InputScanMaskRemove(ADC_ID_1, ADC_INPUT_SCAN_AN10);
  PLIB_ADC_SampleAcquisitionTimeSet(ADC_ID_1, 2);
  PLIB_ADC_ConversionClockSet(ADC_ID_1, 80000000, 20000000);
  PLIB_ADC_Enable(ADC_ID_1);

#if USE_USB==0
  // disable USB peripheral (was enabled in DRV_USBFS_Initialize() called from system_init.c)
  PLIB_USB_Disable(USB_ID_1);

  // set up USART 2 on pins 21/22 (B10/B11) at 750000 baud, 8N1
  c = SYS_CLK_PeripheralFrequencyGet(CLK_BUS_PERIPHERAL_1);
  PLIB_PORTS_PinModePerPortSelect(PORTS_ID_0, PORT_CHANNEL_B, 10, PORTS_PIN_MODE_DIGITAL);
  PLIB_PORTS_PinModePerPortSelect(PORTS_ID_0, PORT_CHANNEL_B, 11, PORTS_PIN_MODE_DIGITAL);
  PLIB_PORTS_RemapOutput(PORTS_ID_0, OUTPUT_FUNC_U2TX, OUTPUT_PIN_RPB10);
  PLIB_PORTS_RemapInput(PORTS_ID_0, INPUT_FUNC_U2RX, INPUT_PIN_RPB11);
  PLIB_USART_InitializeModeGeneral(USART_ID_2, false, false, false, false, false);
  PLIB_USART_LineControlModeSelect(USART_ID_2, USART_8N1);
  PLIB_USART_InitializeOperation(USART_ID_2, USART_RECEIVE_FIFO_ONE_CHAR, USART_TRANSMIT_FIFO_IDLE, USART_ENABLE_TX_RX_USED);
  PLIB_USART_BaudRateHighEnable(USART_ID_2);
  PLIB_USART_BaudRateHighSet(USART_ID_2, c, 750000);
  PLIB_USART_TransmitterEnable(USART_ID_2);
  PLIB_USART_ReceiverEnable(USART_ID_2);
  PLIB_USART_Enable(USART_ID_2);
#else
  // set up USB
  USB_HOST_CDC_AttachEventHandlerSet(USBHostCDCAttachEventListener, (uintptr_t) 0);
  PLIB_USB_StopInIdleDisable(USB_ID_1);
  USB_HOST_BusEnable(0);
#endif
    
  // set color/gray scale output pin
  ColorStateSet((dazzler_picture_ctrl & 0x10)!=0);
    
  // clear frame buffer
  memset(framebuffer, 0, sizeof(framebuffer));

  // determine whether to enter joystick calibration (test) mode
  test_mode = read_joystick_buttons();
  if( (test_mode & 0x0f)!=0x0f )
    test_mode = 1;
  else if( (test_mode & 0xf0)!=0xf0 )
    test_mode = 2;
  else if( !TestButtonStateGet() )
    test_mode = 13;
  else
    test_mode = 0;

  // if we're in test mode, draw the test screen
  if( test_mode>0 ) { dazzler_ctrl = 0x80; draw_test_screen(); }
    
  // start timer
  if( ALWAYS_ON>0 || test_mode>0 ) PLIB_TMR_Start(TMR_ID_2);
}


// -----------------------------------------------------------------------------
// --------------------------------- main loop ---------------------------------
// -----------------------------------------------------------------------------


void APP_Tasks ( void )
{
  // Displaying the picture and receiving data is done in interrupts
  // so all we do in the main loop is querying the joysticks (once per frame)
  // and processing received data
  check_joystick();

  // process received data    
  ringbuffer_dequeue();
  ringbuffer_dequeue();
  ringbuffer_dequeue();

#if USE_USB>0
  // handle USB tasks
  usbTasks();
  if( test_mode==11 ) framebuffer[0][0] = (usbCdcObject==NULL) ? 9 : 10;
#endif

  // handle test mode switching
  if( test_mode>10 ) check_test_button();
}
