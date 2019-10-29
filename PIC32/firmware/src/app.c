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

// If 1, visualize the current ringbuffer usage on the top of the screen
// If 0, do not visualize
#define SHOW_RINGBUFFER 0


// The following Microchip USB host stack source files have been modified from their original:
//
// File: firmware\src\system_config\default\framework\usb\src\dynamic\usb_host_cdc.c
// line 790: changed USB_CDC_PROTOCOL_AT_V250 to USB_CDC_PROTOCOL_NO_CLASS_SPECIFIC
// (Arduino Due reports CDC protocol as NO_CLASS_SPECIFIC)
//
// File: firmware\src\system_config\default\framework\usb\src\dynamic\usb_host.c
// added line 5391: if( transferType==USB_TRANSFER_TYPE_BULK && maxPacketSize>64 ) maxPacketSize=64;
// (Arduino Due always sends the high speed descriptor when configuration is requested, should
// send full speed descriptor if the host is full speed. Will send full speed descriptor when OTHER 
// configuration is requested. maxPacketSize for full speed bulk transfer is 64 bytes, high speed is 512)


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
#define NUM_LINES     630               // =16.59ms/frame (spec: 16.579ms)
#define VFP_LENGTH    (1+45)            // front porch plus margin
#define VBP_LENGTH    (23+45) 	        // back porch plus margin
#define VSYNC_LENGTH  4                 // =0.10567ms (spec: 0.1056ms)
#define DISPLAY_LINES (128*4)

// sanity check
#if (VFP_LENGTH+VBP_LENGTH+VSYNC_LENGTH+DISPLAY_LINES) != NUM_LINES
#error Inconsistent vertical timing!
#endif


// current scan line
volatile uint32_t g_current_line = 0, g_frame_ctr = 0;

// flag to indicate to main function that VSYNC should be sent
volatile bool send_vsync = false;

// Dazzler control register:
// D7: on/off
// D6-D0: screen memory location (not used in client)
uint8_t dazzler_ctrl = 0x00;

// Dazzler picture control register:
// D7: not used
// D6: 1=resolution x4, 0=normal resolution
// D5: 1=2k memory, 0=512byte memory
// D4: 1=color, 0=monochrome
// D3-D0: foreground color for x4 high res mode
uint8_t dazzler_picture_ctrl = 0x10;

// dazzler video memory, keeping two buffers plus one current-frame buffer
uint8_t dazzler_mem[2 * 2048], dazzler_mem_buf[2048];

// test mode (see function draw_test_screen)
int test_mode = 0;

// computer/dazzler version
#define DAZZLER_VERSION 0x01
int computer_version =  0x00;

// dazzler commands received from the Altair simulator
#define DAZ_MEMBYTE   0x10
#define DAZ_FULLFRAME 0x20
#define DAZ_CTRL      0x30
#define DAZ_CTRLPIC   0x40
#define DAZ_VERSION   0xF0

// dazzler commands sent to the Altair simulator
#define DAZ_JOY1      0x10
#define DAZ_JOY2      0x20
#define DAZ_KEY       0x30
#define DAZ_VSYNC     0x40

// receiver states
#define ST_IDLE      0
#define ST_MEMBYTE1  1
#define ST_MEMBYTE2  2
#define ST_CTRL      3
#define ST_CTRLPIC   4
#define ST_FULLFRAME 5


static void dazzler_send(uint8_t *buffer, size_t len);

// -----------------------------------------------------------------------------
// --------------------------- ring-buffer handling ----------------------------
// -----------------------------------------------------------------------------


#define RINGBUFFER_SIZE 0x01000 // must be a power of 2
volatile uint32_t ringbuffer_start = 0, ringbuffer_end = 0;
uint8_t ringbuffer[RINGBUFFER_SIZE];

#define ringbuffer_full()                (((ringbuffer_end+1)&(RINGBUFFER_SIZE-1)) == ringbuffer_start)
#define ringbuffer_empty()                 (ringbuffer_start==ringbuffer_end)
#define ringbuffer_available_for_read()  (((ringbuffer_end+RINGBUFFER_SIZE)-ringbuffer_start)&(RINGBUFFER_SIZE-1))
#define ringbuffer_available_for_write() (((ringbuffer_start+RINGBUFFER_SIZE)-ringbuffer_end-1)&(RINGBUFFER_SIZE-1))
#define ringbuffer_peek()                  (ringbuffer[ringbuffer_start])


inline void ringbuffer_enqueue(uint8_t b)
{
  ringbuffer[ringbuffer_end] = b;
  ringbuffer_end = (ringbuffer_end+1) & (RINGBUFFER_SIZE-1);
}

inline uint8_t ringbuffer_dequeue()
{
  uint8_t data = ringbuffer[ringbuffer_start];
  ringbuffer_start = (ringbuffer_start+1) & (RINGBUFFER_SIZE-1);
  return data;
}

uint8_t ringbuffer_process_data()
{
  static uint32_t addr = 0, cnt = 0;
  uint32_t available;
  uint8_t cmd;

  available = ringbuffer_available_for_read();
  cmd = ringbuffer_peek();

  if( cnt==0 && available>0 )
    {
      switch( cmd & 0xF0 )
        {
        case DAZ_MEMBYTE: 
          {
            if( available>=3 )
              {
                ringbuffer_dequeue();
                addr = (cmd & 0x0F) * 256 + ringbuffer_dequeue();
                dazzler_mem[addr] = ringbuffer_dequeue();
              }
            break;
          }

        case DAZ_CTRL:
          {
            if( (cmd&0x0F)!=0 )
              {
                // illegal command
                ringbuffer_dequeue();
              }
            else if(  available>=2 )
              {
                ringbuffer_dequeue();
                dazzler_ctrl = ringbuffer_dequeue();
                // a version 0 computer writes only to buffer 0 but may set bit 0
                // (used in version 1+ as buffer-select) as either 0 or 1.
                if( computer_version==0 ) dazzler_ctrl &= 0xFE;
                test_mode = 0;
#if ALWAYS_ON==0
                // start/stop generating the output signal
                g_current_line = 0;
                if( dazzler_ctrl & 0x80 )
                  PLIB_TMR_Start(TMR_ID_2);
                else
                  PLIB_TMR_Stop(TMR_ID_2);
#endif      
              }
            break;
          }
            
        case DAZ_CTRLPIC:
          {
            if( (cmd&0x0F)!=0 )
              {
                // illegal command
                ringbuffer_dequeue();
              }
            else if( available>=2 )
              {
                ringbuffer_dequeue();
                dazzler_picture_ctrl = ringbuffer_dequeue();
              }
            break;
          }
            
        case DAZ_FULLFRAME:
          {
            ringbuffer_dequeue();
            
            // only a valid FULLFRAME command if bits 1+2 are zero
            if( (cmd&0x06)==0 )
              {
                addr  = (cmd & 0x08) * 256;
                cnt   = (cmd & 0x01) ? 2048 : 512;
                if( available>0 ) available--;
              }
            break;
          }

        case DAZ_VERSION:
          {
            ringbuffer_dequeue();
            computer_version = cmd & 0x0F;

            // respond by sending our version to the computer
            static uint8_t cmd = DAZ_VERSION | (DAZZLER_VERSION&0x0F);
            dazzler_send(&cmd, 1);
            break;
          }
        
        default:
          {
            // remove the unrecognized command from the ringbuffer, otherwise
            // we would just block forever. Given that we ignore unrecognized
            // commands, there is a chance we'll get back into sync.
            ringbuffer_dequeue();
            break;
        }
      }
  }

  if( cnt>0 && available>0 )
    {
      // receiving fullframe data
      uint32_t n = ringbuffer_start<=ringbuffer_end ? ringbuffer_end-ringbuffer_start : RINGBUFFER_SIZE-ringbuffer_start;
      n = min(n, cnt);
      memcpy(dazzler_mem+addr, ringbuffer+ringbuffer_start, n);
      addr += n;
      cnt  -= n;
      ringbuffer_start = (ringbuffer_start+n) & (RINGBUFFER_SIZE-1);
   }
}


// -----------------------------------------------------------------------------
// ----------------------------- test mode handling ----------------------------
// -----------------------------------------------------------------------------


int get_pixel_128x128(int x, int y)
{
  uint32_t addr;
  static uint8_t bitmasks[8] = {0x01, 0x02, 0x10, 0x20, 0x04, 0x08, 0x40, 0x80};

  x &= 127;
  y &= 127;
  addr = (y&62)*8 + ((x&63)/4);
  if( x>=64 ) addr += 512;
  if( y>=64 ) addr += 1024;

  return (dazzler_mem[addr] & (bitmasks[(x&3) + 4*(y&1)])) ? 1 : 0;
}


void set_pixel_128x128(int x, int y, int on)
{
  uint32_t addr;
  static uint8_t bitmasks[8] = {0x01, 0x02, 0x10, 0x20, 0x04, 0x08, 0x40, 0x80};

  x &= 127;
  y &= 127;
  addr = (y&62)*8 + ((x&63)/4);
  if( x>=64 ) addr += 512;
  if( y>=64 ) addr += 1024;

  if( on )
    dazzler_mem[addr] |=  (bitmasks[(x&3) + 4*(y&1)]);
  else 
    dazzler_mem[addr] &= ~(bitmasks[(x&3) + 4*(y&1)]);
}


void draw_joystick_pixel(int x, int y)
{
  static int px, py, pc = -1;
  x = x/2+64;
  y = 63-y/2;
  if( pc>=0 ) set_pixel_128x128(px, py, pc);
  pc = get_pixel_128x128(x, y);
  px = x;
  py = y;
  set_pixel_128x128(x, y, 1);
}


void draw_joystick_buttons(uint8_t buttons)
{
  set_pixel_128x128(61, 0, buttons & 0x01);
  set_pixel_128x128(63, 0, buttons & 0x02);
  set_pixel_128x128(65, 0, buttons & 0x04);
  set_pixel_128x128(67, 0, buttons & 0x08);
}


void draw_test_screen()
{
  int r, c;
  switch( test_mode )
    {
    case 1:
    case 2:
      // joystick test screen
      dazzler_picture_ctrl = 0x79;
      for(r=0; r<128; r++)
        {
          set_pixel_128x128(  0, r, 0x01);
          set_pixel_128x128( 64, r, (r & 1) && r<127 ? 0x01 : 0x00);
          set_pixel_128x128(127, r, 0x01);
        }
      for(c=0; c<128; c++)
        {
          set_pixel_128x128(c,   0, 0x01);
          set_pixel_128x128(c,  63, (c & 1) && c<127 ? 0x00 : 0x01);
          set_pixel_128x128(c, 127, 0x01);
        }
      break;
                        
    case 11:
      // color test pattern (normal res, small mem)
      dazzler_picture_ctrl = 0x10;
      for(r=0; r<32; r++)
        for(c=0; c<16; c++)
          dazzler_mem[r*16+c] = ((r+c)&7) + 16*(((r+c)&7)+8);
      break;
                        
    case 12:
      // color test pattern (normal res, big mem)
      dazzler_picture_ctrl = 0x30;
      for(r=0; r<32; r++)
        for(c=0; c<16; c++)
          dazzler_mem[r*16+c] = ((r+c)&7) + 16*(((r+c)&7)+8);
      memcpy(dazzler_mem+512, dazzler_mem, 512);
      memcpy(dazzler_mem+1024, dazzler_mem, 1024);
      break;

    case 13:
      // black-and-white grid pattern (4x res, small mem)
      dazzler_picture_ctrl = 0x4F; 
      for(r=0; r<512; r++) dazzler_mem[r] = 0xEE;
      break;
                        
    case 14:
      // black-and-white grid pattern (4x res, big mem)
      dazzler_picture_ctrl = 0x6F; 
      for(r=0; r<2048; r++) dazzler_mem[r] = 0xEE;
      break;
                        
    case 15:
      // gray-scale test pattern (normal res, small mem)
      dazzler_picture_ctrl = 0x00;
      for(r=0; r<32; r++)
        for(c=0; c<16; c++)
          dazzler_mem[r*16+c] = ((r+c*2)&15) + 16*((r+c*2+1)&15);
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
      if( test_mode>15 ) test_mode = 11;
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
            // check if there are any any changes for joystick 1
            if( x1!=x1p || y1!=y1p || b1 != b1p )
              {
                // send joystick 1 data
                static uint8_t buf[3];
                buf[0] = DAZ_JOY1 | b1;
                buf[1] = x1;
                buf[2] = y1;
                dazzler_send(buf, 3);

                // remember current values
                x1p = x1; y1p = y1; b1p = b1;
              }

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
            // check if there are any any changes for joystick 2
            if( x2!=x2p || y2!=y2p || b2 != b2p )
              {
                // send joystick 2 data
                static uint8_t buf[3];
                buf[0] = DAZ_JOY2 | b2;
                buf[1] = x2;
                buf[2] = y2;
                dazzler_send(buf, 3);

                // remember current values
                x2p = x2; y2p = y2; b2p = b2;
              }

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
// -------------------------   video line rendering   --------------------------
// -----------------------------------------------------------------------------

// Line buffer has 2 sets of 2 lines of 128+1 columns (one extra 0 at the end)
// (use 132 bytes per line to stay 32-bit aligned)
// We render two lines at once because of the Dazzler's video memory layout
// where (in x4 resolution mode) one byte contains information for two lines.
// We have two sets of two lines so we can actually display one set while
// rendering the other
#define LL (128+4)
uint8_t linebuffer[4*LL] __attribute__((aligned(32)));

// Each s scan line needs to be repeated 2^repeat_line times
// (this depends on the graphics mode)
uint8_t repeat_line = 0;

// common foreground color to use for the current frame
uint8_t dazzler_fg_color = 0x00;

// line rendering function to use for the current frame
void render_line_dummy(int buffer, int line, int part) {}
void (*render_line)(int, int, int) = &render_line_dummy;

void render_line_bigmem_single(int buffer, int line, int part)
{
  // 2K RAM, high (x4) resolution (128x128 pixels, single foreground color)
  // rendering 2 lines at a time
  uint8_t i, color = dazzler_fg_color;
  uint8_t *lp = linebuffer + (buffer==0 ? 0 : (2*LL)) + part * 16;
  uint8_t *mp = dazzler_mem_buf + (line&62)*8 + (line&64)*16 + (part&3)*4 + (part&4)*128;

  for(i=0; i<4; i++)
    {
      uint8_t b = mp[i];
      lp[0   ] = b & 0x01 ? color : 0;
      lp[1   ] = b & 0x02 ? color : 0;
      lp[2   ] = b & 0x10 ? color : 0;
      lp[3   ] = b & 0x20 ? color : 0;
      lp[0+LL] = b & 0x04 ? color : 0;
      lp[1+LL] = b & 0x08 ? color : 0;
      lp[2+LL] = b & 0x40 ? color : 0;
      lp[3+LL] = b & 0x80 ? color : 0;
      lp += 4;
    }
}


void render_line_bigmem_multi(int buffer, int line, int part)
{
  // 2K RAM, low resolution (64x64 pixels, individual color)
  uint8_t i, color1, color2; 
  uint8_t *lp = linebuffer + (buffer==0 ? 0 : (2*LL)) + part * 16;
  uint8_t *mp = dazzler_mem_buf + (line&62)*8 + (line&64)*16 + (part&3)*4 + (part&4)*128;

  for(i=0; i<4; i++)
    {
      uint8_t b = mp[i];
      color1 = b & 0x0f;
      color2 = (b & 0xf0)/16;

      lp[0   ] = lp[1   ] = color1;
      lp[2   ] = lp[3   ] = color2;
      lp[0+LL] = lp[1+LL] = color1;
      lp[2+LL] = lp[3+LL] = color2;
      lp += 4;
    }
}


void render_line_smallmem_single(int buffer, int line, int part)
{
  // 512 bytes RAM, high resolution (64x64 pixels, common color)
  uint8_t i, color = dazzler_fg_color;
  uint8_t *lp = linebuffer + (buffer==0 ? 0 : (2*LL)) + part * 16;
  uint8_t *mp = dazzler_mem_buf + (line & 124) * 4 + part * 2;

  for(i=0; i<2; i++)
    {
      uint8_t b = mp[i];
      lp[0]    = lp[1]    = b & 0x01 ? color : 0;
      lp[2]    = lp[3]    = b & 0x02 ? color : 0;
      lp[4]    = lp[5]    = b & 0x10 ? color : 0;
      lp[6]    = lp[7]    = b & 0x20 ? color : 0;
      lp[0+LL] = lp[1+LL] = b & 0x04 ? color : 0;
      lp[2+LL] = lp[3+LL] = b & 0x08 ? color : 0;
      lp[4+LL] = lp[5+LL] = b & 0x40 ? color : 0;
      lp[6+LL] = lp[7+LL] = b & 0x80 ? color : 0;
      lp += 8;
    }
}


void render_line_smallmem_multi(int buffer, int line, int part)
{
  // 512 bytes RAM, low resolution (32x32 pixels, individual color)
  uint8_t color1, color2, i;
  uint8_t *lp = linebuffer + (buffer==0 ? 0 : (2*LL)) + part * 16;
  uint8_t *mp = dazzler_mem_buf + (line & 124) * 4 + part * 2;

  for(i=0; i<2; i++)
    {
      uint8_t b = mp[i];
      color1 = b & 0x0f;
      color2 = (b & 0xf0)/16;

      lp[0] = lp[1] = lp[2] = lp[3] = color1;
      lp[4] = lp[5] = lp[6] = lp[7] = color2;
      lp[0+LL] = lp[1+LL] = lp[2+LL] = lp[3+LL] = color1;
      lp[4+LL] = lp[5+LL] = lp[6+LL] = lp[7+LL] = color2;
      lp += 8;
    }
}


void set_render_line()
{
  switch( dazzler_picture_ctrl & 0x60 )
    {
    case 0x20 : render_line = render_line_bigmem_multi;    repeat_line = 2; /* =*4 */ break;
    case 0x60 : render_line = render_line_bigmem_single;   repeat_line = 2; /* =*4 */ break;
    case 0x00 : render_line = render_line_smallmem_multi;  repeat_line = 3; /* =*8 */ break;
    case 0x40 : render_line = render_line_smallmem_single; repeat_line = 3; /* =*8 */ break;
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
      // we are in the vertically visible region. Show line
      // line  0,  1,  2,  3 => linebuffer[0]
      // line  4,  5,  6,  7 => linebuffer[1]
      // line  8,  9, 10, 11 => linebuffer[2]
      // line 12, 13, 14, 15 => linebuffer[3]
      // line 16, 17, 18, 19 => linebuffer[0]
      // ...
      uint8_t *ptr = linebuffer + (((g_current_line-VBP_LENGTH)>>repeat_line)&3) * LL;
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
  
  if( g_current_line>=VBP_LENGTH-8 && g_current_line<VBP_LENGTH+DISPLAY_LINES-8 )
    {
      // we are in (or just before) the vertically visible region. 
      // Render part of next line to be shown
      // scan line   0 => render part 0 of line 0 into linebuffer 0 and 1
      // scan line   1 => render part 1 of line 0 into linebuffer 0 and 1
      // ...
      // scan line   7 => render part 7 of line 0 into linebuffer 0 and 1
      // scan line   8 => render part 0 of line 1 into linebuffer 2 and 3
      // scan line   9 => render part 1 of line 1 into linebuffer 2 and 3
      // ...
      // scan line  15 => render part 7 of line 1 into linebuffer 2 and 3
      // scan line  16 => render part 0 of line 2 into linebuffer 0 and 1
      // ...
      uint32_t line = g_current_line - VBP_LENGTH + 8;
      render_line((line>>(repeat_line+1))&1, line/4, line&7);
#if SHOW_RINGBUFFER>0
      if( line==7 )
        {
          uint8_t c1 = ColorStateGet() ? 0x0A : 0x07;
          uint8_t c2 = ColorStateGet() ? 0x09 : 0x00;
          int s = ringbuffer_start * 128 / RINGBUFFER_SIZE;
          int e = ringbuffer_end   * 128 / RINGBUFFER_SIZE;
          if( s==e )
            memset(linebuffer, c1, 128);
          else if( s<e )
          {
            memset(linebuffer,   c1, s);
            memset(linebuffer+s, c2, e-s);
            memset(linebuffer+e, c1, 128-e);
          }
          else
          {
            memset(linebuffer,   c2, e);
            memset(linebuffer+e, c1, s-e);
            memset(linebuffer+s, c2, 128-s);
          }
          linebuffer[s] = c2;
        }
#endif      
    }
  else if( g_current_line==NUM_LINES-3)
    memcpy(dazzler_mem_buf, dazzler_mem + (dazzler_ctrl & 1) * 2048, 1024);
  else if( g_current_line==NUM_LINES-2)
    memcpy(dazzler_mem_buf + 1024, dazzler_mem + (dazzler_ctrl & 1) * 2048 + 1024, 1024);
  
  // increase line counter and roll over when we reach the bottom of the screen
  if( ++g_current_line==NUM_LINES ) 
    { 
      g_current_line=0; 
      g_frame_ctr++; 

      // signal to send VSYNC command to computer (if computer understands it)
      // (can't send directly from here since it can cause lock-ups in USB)
      if( computer_version>0 ) send_vsync = true;
      
      // set the color/grayscale bit for the next frame
      ColorStateSet((dazzler_picture_ctrl & 0x10)!=0);

      // set the render_line function for the next frame
      set_render_line();

      // set the common foreground color for next frame
      dazzler_fg_color = dazzler_picture_ctrl & 0x0F;
 }

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

// technically more than 8 64-byte packets can fit into a 1ms frame but
// the video interrupt occupies a significant amount of time so USB
// handling can get delayed processing the received packets. We only
// request 8 64-byte data packets to leave some safety margin.
#define USB_MAX_TRANSFER_SIZE (8*64) // must be a multiple of 64

static USB_HOST_CDC_OBJ    usbCdcObject     = NULL;
static USB_HOST_CDC_HANDLE usbCdcHostHandle = USB_HOST_CDC_HANDLE_INVALID;
volatile bool usbBusy = false;
uint8_t usbbuffer[USB_MAX_TRANSFER_SIZE];


void usbScheduleRead()
{
  // if a USB read or write request is currently in process then we can't
  // schedule a transfer right now
  if( !usbBusy )
    {
      // If there is space available in the ringbuffer then ask the client
      // to send more data. If there is no space then do not start another
      // request until we have processed some data and enough space is available
      // to store at least one full 64-byte packet of USB traffic
      size_t avail = ringbuffer_available_for_write() & ~0x3F;
      if( avail>0 ) 
        {
          USB_HOST_CDC_Read(usbCdcHostHandle, NULL, usbbuffer, min(avail, USB_MAX_TRANSFER_SIZE));
          usbBusy = true;
        }
    }
}


USB_HOST_CDC_EVENT_RESPONSE USBHostCDCEventHandler(USB_HOST_CDC_HANDLE cdcHandle, USB_HOST_CDC_EVENT event, void * eventData, uintptr_t context)
{
  switch(event)
    {
    case USB_HOST_CDC_EVENT_READ_COMPLETE:
      {
        USB_HOST_CDC_EVENT_READ_COMPLETE_DATA *readCompleteEventData = (USB_HOST_CDC_EVENT_READ_COMPLETE_DATA *)(eventData);
        if( readCompleteEventData->result == USB_HOST_CDC_RESULT_SUCCESS )
          {
            size_t len = readCompleteEventData->length;
            if( ringbuffer_end+len < RINGBUFFER_SIZE )
              {
                memcpy(ringbuffer+ringbuffer_end, usbbuffer, len);
                ringbuffer_end += len;
              }
            else
              {
                size_t len2 = RINGBUFFER_SIZE-ringbuffer_end;
                memcpy(ringbuffer+ringbuffer_end, usbbuffer, len2);
                memcpy(ringbuffer, usbbuffer+len2, len-len2);
                ringbuffer_end = len-len2;
              }
          }

        // schedule another read
        usbBusy = false;
        usbScheduleRead();
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


void USBHostCDCAttachEventListener(USB_HOST_CDC_OBJ cdcObj, uintptr_t context)
{
  // a client has been attached
  usbCdcObject = cdcObj;
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

              // set the line coding: 115200 baud 8N1
              // These settings will be ignored if connected to the
              // Due's Native USB port but will be used when connected to
              // the Programming USB port, so the port in the Simulator needs
              // to be set accordingly. The baud rate of 115200 is the highest
              // baud rate for which the 16U2 serial-to-usb converter can sustain
              // bi-directional traffic without introducing errors, see here:
              // https://github.com/arduino/ArduinoCore-avr/issues/296
              static USB_CDC_LINE_CODING coding = {115200, 0, 0, 8};
              USB_HOST_CDC_ACM_LineCodingSet(usbCdcHostHandle, NULL, &coding);

              // initialize ringbuffer
              ringbuffer_start = ringbuffer_end = 0;
              computer_version = 0;
              usbBusy = false;
            }
        }
    }
  else
    {
      // we have a connection => schedule a new read if none is currently going
      // (can't allow USB interrupts while scheduling a new transfer)
      PLIB_USB_InterruptDisable(USB_ID_1, USB_INT_TOKEN_DONE);
      usbScheduleRead();
      PLIB_USB_InterruptEnable(USB_ID_1, USB_INT_TOKEN_DONE);
    }
}
#endif


static void dazzler_send(uint8_t *buffer, size_t len)
{
#if USE_USB>0
  if( usbCdcHostHandle!=USB_HOST_CDC_HANDLE_INVALID )
    USB_HOST_CDC_Write(usbCdcHostHandle, NULL, (void *) buffer, len);
#else
  size_t i;
  for(i=0; i<len; i++) PLIB_USART_TransmitterByteSend(USART_ID_2, buffer[i]);
#endif
}


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

  // clear line rendering buffer
  memset(linebuffer, 0, sizeof(linebuffer));
    
  // determine whether to enter joystick calibration (test) mode
  test_mode = read_joystick_buttons();
  if( (test_mode & 0x0f)!=0x0f )
    test_mode = 1;
  else if( (test_mode & 0xf0)!=0xf0 )
    test_mode = 2;
  else if( !TestButtonStateGet() )
    test_mode = 15;
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
  // query joystick (in first 7 lines of the vertical back porch)
  check_joystick();

  // process received data    
  ringbuffer_process_data();

#if USE_USB>0
  // handle USB tasks
  usbTasks();
  if( test_mode==11 ) dazzler_mem[0] = ((usbCdcObject==NULL) ? 9 : 10) + (dazzler_mem[0] & 0xF0);
#else
  // receive serial data
  // There's really not much we can do if we receive a byte of data
  // when the ring buffer is full. Overwriting the beginning of the buffer
  // is about as bad as dropping the newly received byte. So we save
  // the time to check whether the buffer is full and just overwrite.
  while( PLIB_USART_ReceiverDataIsAvailable(USART_ID_2) ) 
    ringbuffer_enqueue(PLIB_USART_ReceiverByteReceive(USART_ID_2));
#endif

  // check if we need to send VSYNC to the host
  if( send_vsync )
  {
     static uint8_t vsync = DAZ_VSYNC;
     dazzler_send(&vsync, 1);
     send_vsync = false;
  }
  
  // handle test mode switching
  check_test_button();
}
