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

#define DAZ_MEMBYTE   0x10
#define DAZ_FULLFRAME 0x20
#define DAZ_CTRL      0x30
#define DAZ_CTRLPIC   0x40

#define DAZ_JOY1      0x10
#define DAZ_JOY2      0x20
#define DAZ_KEY       0x30

COLORREF colors[16] = {RGB(0x00,0x00,0x00), RGB(0x80,0x00,0x00), RGB(0x00,0x80,0x00), RGB(0x80,0x80,0x00),
                       RGB(0x00,0x00,0x80), RGB(0x80,0x00,0x80), RGB(0x00,0x80,0x80), RGB(0x80,0x80,0x80),
                       RGB(0x00,0x00,0x00), RGB(0xff,0x00,0x00), RGB(0x00,0xff,0x00), RGB(0xff,0xff,0x00),
                       RGB(0x00,0x00,0xff), RGB(0xff,0x00,0xff), RGB(0x00,0xff,0xff), RGB(0xff,0xff,0xff)};



// D7: not used
// D6: 1=resolution x4, 0=normal resolution
// D5: 1=2k memory, 0=512byte memory
// D4: 1=color, 0=monochrome
// D3-D0: color info for x4 high res mode
byte picture_ctrl = 0;


// D7: on/off
// D6-D0: screen memory location (not used in client)
byte ctrl = 0;

HBRUSH brushes_color[16], brushes_grayscale[16];

byte dazzler_mem[2048];

int    g_com_port = -1;
int    g_com_baud = 1050000;
HANDLE serial_conn = INVALID_HANDLE_VALUE;

wchar_t *peer = NULL;
SOCKET server_socket = INVALID_SOCKET;

HANDLE draw_mutex = INVALID_HANDLE_VALUE;

int pixel_scaling = 1, border_topbottom = 0, border_leftright = 0;
int joy1_x = 0, joy1_y = 0, joy1_buttons = 0x0f;


static void draw_pixel(HDC dc, byte x, byte y, byte w, byte color)
{
  RECT r;
  r.left   = border_leftright + x     * pixel_scaling * w;
  r.right  = border_leftright + (x+1) * pixel_scaling * w;
  r.top    = border_topbottom + y     * pixel_scaling * w;
  r.bottom = border_topbottom + (y+1) * pixel_scaling * w;
  FillRect(dc, &r, (picture_ctrl & 0x10) ? (brushes_color[color & 15]) : (brushes_grayscale[color & 15]));
}


static void update_byte(HDC dc, int i)
{
  int x, y;
  byte w = 1, b = dazzler_mem[i];

  // determine position within quadrant
  x = (i & 0x000f)*2;
  y = (i & 0x01f0)/16;

  if( picture_ctrl & 0x20 )
    {
      // using 2K memory => determine quadrant
      if( i & 0x0200 ) x += 32;
      if( i & 0x0400 ) y += 32; 
    }
  else
    {
      // using 0.5K memory => scale pixel size up
      w *= 2;
    }

  if( picture_ctrl & 0x40 )
    {
      // high-resolution, common color
      x *= 2;
      y *= 2;

      byte color = picture_ctrl & 0x0f;
      draw_pixel(dc, x,   y,   w, b & 0x01 ? color : 0);
      draw_pixel(dc, x+1, y,   w, b & 0x02 ? color : 0);
      draw_pixel(dc, x,   y+1, w, b & 0x04 ? color : 0);
      draw_pixel(dc, x+1, y+1, w, b & 0x08 ? color : 0);
      draw_pixel(dc, x+2, y,   w, b & 0x10 ? color : 0);
      draw_pixel(dc, x+3, y,   w, b & 0x20 ? color : 0);
      draw_pixel(dc, x+2, y+1, w, b & 0x40 ? color : 0);
      draw_pixel(dc, x+3, y+1, w, b & 0x80 ? color : 0);
    }
  else
    {
      // low-resolution, individual color
      w *= 2;

      draw_pixel(dc, x,   y, w,  b & 0x0f);
      draw_pixel(dc, x+1, y, w, (b & 0xf0)/16);
    }
}


static void update_frame(HWND hwnd)
{
  WaitForSingleObject(draw_mutex, INFINITE);

  HDC hdc = GetDC(hwnd);
  if( ctrl & 0x80 ) 
    {
      int w = (picture_ctrl & 0x20) ? 2048 : 512;
      for(int i=0; i<w; i++) update_byte(hdc, i);
    }
  else
    {
      RECT r;
      GetClientRect(hwnd, &r);
      FillRect(hdc, &r, brushes_color[0]);
    }
  ReleaseDC(hwnd, hdc);

  ReleaseMutex(draw_mutex);
}


void set_window_title(HWND hwnd)
{
  bool connected = (serial_conn!=INVALID_HANDLE_VALUE) || (server_socket!=INVALID_SOCKET);
  bool on = (ctrl & 0x80)!=0;
  wchar_t buf[100];

  if( peer!=NULL )
    wsprintf(buf, L"Dazzler Display (%s, %sconnected, %s)",
             peer, connected ? L"" : L"not ", on ? L"on" : L"off");
  else if( g_com_port>0 )
    wsprintf(buf, L"Dazzler Display (COM%i, %sconnected, %s)",
             g_com_port, connected ? L"" : L"not ", on ? L"on" : L"off");
  else
    wsprintf(buf, L"Dazzler Display");
  
  SetWindowText(hwnd, buf);
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
                case DAZ_FULLFRAME: 
                  if( ctrl & 0x80 ) update_frame(hwnd);
                  break;
                  
                case DAZ_MEMBYTE:
                  {
                    int a = buf[0]*256+buf[1];
                    dazzler_mem[a] = buf[2];

                    if( ctrl & 0x80 ) 
                      {
                        WaitForSingleObject(draw_mutex, INFINITE);
                        HDC hdc = GetDC(hwnd);
                        update_byte(hdc, a);
                        ReleaseDC(hwnd, hdc);
                        ReleaseMutex(draw_mutex);
                      }

                    break;
                  }
                  
                case DAZ_CTRL:
                  ctrl = buf[0];
                  update_frame(hwnd);
                  set_window_title(hwnd);
                  break;

                case DAZ_CTRLPIC:
			      if( picture_ctrl != buf[0] )
				  {
                    picture_ctrl = buf[0];
                    update_frame(hwnd);
				  }
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
            case DAZ_FULLFRAME: 
              recv_bytes = (data[i] & 0x0f) ? 2048 : 512;
              break;
                  
            case DAZ_MEMBYTE:
              recv_bytes = 2;
              buf[recv_ptr++] = data[i]&0x07;
              break;

            case DAZ_CTRL:
            case DAZ_CTRLPIC:
              recv_bytes = 1;
              break;

            default:
              recv_status = 0;
              break;
            }

          i++;
        }
    }
}


void dazzler_send(HWND hwnd, byte *data, int size)
{
  DWORD n;

  if( serial_conn!=INVALID_HANDLE_VALUE )
    WriteFile(serial_conn, data, size, &n, NULL);
  if( server_socket!=INVALID_SOCKET )
    send(server_socket, (char *) data, size, 0);
}


enum
  {
    ID_SOCKET = WM_USER,
    ID_FULLSCREEN,
    ID_ABOUT,
    ID_EXIT,
    ID_BAUD_9600,
    ID_BAUD_38400,
    ID_BAUD_115200,
    ID_BAUD_250000,
    ID_BAUD_525000,
    ID_BAUD_750000,
    ID_BAUD_1050000,
    ID_PORT_NONE,
    ID_PORT
  };


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


void write_settings(int port, int baud)
{
  HKEY key;
  if( RegCreateKeyEx(HKEY_CURRENT_USER, L"Software\\DazzlerDisplay", 0, NULL, REG_OPTION_NON_VOLATILE, KEY_SET_VALUE, NULL, &key, NULL) == ERROR_SUCCESS )
    {
      RegSetValueEx(key, L"Port", 0, REG_DWORD, (const LPBYTE) &port, 4);
      RegSetValueEx(key, L"Baud", 0, REG_DWORD, (const LPBYTE) &baud, 4);
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
      if( RegQueryValueEx(key, L"Port", 0, &tp, (LPBYTE) port, &l) == ERROR_SUCCESS && tp==REG_DWORD ) 
        if( RegQueryValueEx(key, L"Baud", 0, &tp, (LPBYTE) baud, &l) == ERROR_SUCCESS && tp==REG_DWORD ) 
          res = true;

      RegCloseKey(key);
    }

  return res;
}


void set_baud_rate(HWND hwnd, int baud)
{
  int id;
  if( baud<=9600         ) id = ID_BAUD_9600;
  else if( baud<=38400   ) id = ID_BAUD_38400;
  else if( baud<=115200  ) id = ID_BAUD_115200;
  else if( baud<=250000  ) id = ID_BAUD_250000;
  else if( baud<=525000  ) id = ID_BAUD_525000;
  else if( baud<=750000  ) id = ID_BAUD_750000;
  else                     id = ID_BAUD_1050000;

  g_com_baud = baud;
  HMENU menuBaud = GetSubMenu(GetSubMenu(GetMenu(hwnd), 2), 1);
  CheckMenuRadioItem(menuBaud, ID_BAUD_9600, ID_BAUD_1050000, id, MF_BYCOMMAND);
  write_settings(g_com_port, g_com_baud);
}


void set_com_port(HWND hwnd, int port)
{
  g_com_port = port;

  HMENU menuPort = GetSubMenu(GetSubMenu(GetMenu(hwnd), 2), 0);
  if( !CheckMenuRadioItem(menuPort, ID_PORT-1, ID_PORT+255, ID_PORT+g_com_port, MF_BYCOMMAND) )
    {
      // checking the item failed => no such port
      g_com_port = -1;
      CheckMenuRadioItem(menuPort, ID_PORT-1, ID_PORT+255, ID_PORT+g_com_port, MF_BYCOMMAND);
    }

  set_window_title(hwnd);
  if( g_com_port>0 ) write_settings(g_com_port, g_com_baud);
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


  int new_port = -1;
  HMENU menuPort = CreateMenu();
  AppendMenu(menuPort, MF_BYPOSITION | MF_STRING, ID_PORT_NONE, L"None");
  for(int i=0; i<256; i++)
    {
      if( found_port[i] )
        {
          // port was found in this check
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
          AppendMenu(menuPort, MF_BYPOSITION | MF_STRING, ID_PORT+i, comName);
        }

      known_port[i] = found_port[i];
    }
  
  HMENU menuSettings = GetSubMenu(GetMenu(hwnd), 2);
  ModifyMenu(menuSettings, 0, MF_BYPOSITION|MF_POPUP, (UINT_PTR) menuPort, L"&Port");
  if( !first_run && new_port>0 ) 
    set_com_port(hwnd, new_port);
  else
    CheckMenuRadioItem(menuPort, ID_PORT-1, ID_PORT+255, ID_PORT+g_com_port, MF_BYCOMMAND);

  first_run = false;
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
              if( dwRead>0 && hwnd!=NULL ) dazzler_receive(hwnd, buf, dwRead);
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


void calc_pixel_scaling(HWND hwnd)
{
  RECT r;
  GetClientRect(hwnd, &r);
  int h = r.bottom-r.top, w = r.right-r.left;

  pixel_scaling = w < h ? w/128 : h/128;
  border_topbottom = (h-(pixel_scaling*128))/2;
  border_leftright = (w-(pixel_scaling*128))/2;
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
}


void toggle_fullscreen(HWND hwnd)
{
  static WINDOWPLACEMENT s_wpPrev = { sizeof(s_wpPrev) };
  static HMENU s_menu = NULL;

  DWORD dwStyle = GetWindowLong(hwnd, GWL_STYLE);
  if( dwStyle & WS_OVERLAPPEDWINDOW )
    {
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
        }
    } 
  else 
    {
      SetWindowLong(hwnd, GWL_STYLE,dwStyle | WS_OVERLAPPEDWINDOW);
      SetWindowPlacement(hwnd, &s_wpPrev);
      SetWindowPos(hwnd, NULL, 0, 0, 0, 0,SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER |SWP_NOOWNERZORDER | SWP_FRAMECHANGED);
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
          case ID_EXIT: 
            PostQuitMessage(0); 
            break;

          case ID_FULLSCREEN: 
            toggle_fullscreen(hwnd); 
            break;

          case ID_BAUD_9600:    set_baud_rate(hwnd, 9600); break;
          case ID_BAUD_38400:   set_baud_rate(hwnd, 38400); break;
          case ID_BAUD_115200:  set_baud_rate(hwnd, 115200); break;
          case ID_BAUD_250000:  set_baud_rate(hwnd, 250000); break;
          case ID_BAUD_525000:  set_baud_rate(hwnd, 525000); break;
          case ID_BAUD_750000:  set_baud_rate(hwnd, 750000); break;
          case ID_BAUD_1050000: set_baud_rate(hwnd, 1050000); break;

          case ID_ABOUT:
            MessageBox(hwnd, 
                       L"Cromemco Dazzler Display application for\nArduino Altair 88000 simulator\n\n"
                       L"https://www.hackster.io/david-hansel/arduino-altair-8800-simulator-3594a6\n\n"
                       L"(C) 2018 David Hansel", 
                       L"About", MB_OK | MB_ICONINFORMATION);
            break;

          default:
            {
              if( id>=ID_PORT_NONE && id<ID_PORT+256 )
                set_com_port(hwnd, id-ID_PORT);
              break;
            }
          }
        break;
      }

    case WM_SIZE:
      {
        RECT r;
        HDC hdc;

        WaitForSingleObject(draw_mutex, INFINITE);
        hdc = GetDC(hwnd);
        GetClientRect(hwnd, &r);
        FillRect(hdc, &r, brushes_color[0]);
        ReleaseDC(hwnd, hdc);
        calc_pixel_scaling(hwnd);
        ReleaseMutex(draw_mutex);

        update_frame(hwnd);
        break;
      }
      
    case WM_CHAR: 
      {
        if( wParam==6 || (wParam==27 && (GetWindowLong(hwnd, GWL_STYLE)&WS_OVERLAPPEDWINDOW)==0) )
          toggle_fullscreen(hwnd);

        byte msg[2];
        msg[0] = DAZ_KEY;
        msg[1] = wParam;
        dazzler_send(hwnd, msg, 2);
        break;
      }
      
    case WM_LBUTTONDBLCLK:
      {
        toggle_fullscreen(hwnd);
        break;
      }

    case WM_PAINT:
      {
        WaitForSingleObject(draw_mutex, INFINITE);
        PAINTSTRUCT ps;
        HDC hdc = BeginPaint(hwnd, &ps);
        EndPaint(hwnd, &ps);
        ReleaseMutex(draw_mutex);
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

        if( uMsg==MM_JOY1MOVE || uMsg==MM_JOY1BUTTONUP || uMsg==MM_JOY1BUTTONDOWN ) 
          msg[0] = DAZ_JOY2;
        else
          msg[0] = DAZ_JOY1;

        if( !(wParam & JOY_BUTTON1) ) msg[0] |= 1;
        if( !(wParam & JOY_BUTTON2) ) msg[0] |= 2;
        if( !(wParam & JOY_BUTTON3) ) msg[0] |= 4;
        if( !(wParam & JOY_BUTTON4) ) msg[0] |= 8;

        msg[1] = get_joy_value(LOWORD(lParam));
        msg[2] = get_joy_value(0xFFFF-HIWORD(lParam));

        dazzler_send(hwnd, msg, 3);
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
    // Register the window class.
    const wchar_t CLASS_NAME[]  = L"Dazzler Window Class";

    draw_mutex = CreateMutex(NULL, FALSE, NULL);
    
    WNDCLASS wc = { };

    wc.lpfnWndProc   = WindowProc;
    wc.hInstance     = hInstance;
    wc.lpszClassName = CLASS_NAME;
    wc.style         = CS_DBLCLKS;
    wc.hCursor       = LoadCursor(NULL, IDC_ARROW);

    RegisterClass(&wc);

    // Create the window.
    long w = 128*4, h = 128*4;
    calc_window_size(&w, &h);
    HWND hwnd = CreateWindowEx(0, CLASS_NAME, L"Dazzler Display", WS_OVERLAPPEDWINDOW,
                               CW_USEDEFAULT, CW_USEDEFAULT, w, h,
                               NULL, NULL, hInstance, NULL);
    
    if( hwnd == NULL )
      return 0;

    HMENU menu = CreateMenu();
    HMENU menuFile = CreateMenu();
    AppendMenu(menuFile, MF_BYPOSITION | MF_STRING, ID_EXIT, L"E&xit");
    HMENU menuView = CreateMenu();
    AppendMenu(menuView, MF_BYPOSITION | MF_STRING, ID_FULLSCREEN, L"&Full Screen\tCtrl+F");
    HMENU menuPort = CreateMenu();
    AppendMenu(menuPort, MF_BYPOSITION | MF_STRING, ID_PORT_NONE, L"None");
    HMENU menuBaud = CreateMenu();
    AppendMenu(menuBaud, MF_BYPOSITION | MF_STRING, ID_BAUD_9600, L"9600");
    AppendMenu(menuBaud, MF_BYPOSITION | MF_STRING, ID_BAUD_38400, L"38400");
    AppendMenu(menuBaud, MF_BYPOSITION | MF_STRING, ID_BAUD_115200, L"115200");
    AppendMenu(menuBaud, MF_BYPOSITION | MF_STRING, ID_BAUD_250000, L"250000");
    AppendMenu(menuBaud, MF_BYPOSITION | MF_STRING, ID_BAUD_525000, L"525000");
    AppendMenu(menuBaud, MF_BYPOSITION | MF_STRING, ID_BAUD_750000, L"750000");
    AppendMenu(menuBaud, MF_BYPOSITION | MF_STRING, ID_BAUD_1050000, L"1050000");
    HMENU menuSettings = CreateMenu();
    AppendMenu(menuSettings, MF_POPUP, (UINT_PTR) menuPort, L"&Port");
    AppendMenu(menuSettings, MF_POPUP, (UINT_PTR) menuBaud, L"&Baud Rate");
    HMENU menuHelp = CreateMenu();
    AppendMenu(menuHelp, MF_BYPOSITION | MF_STRING, ID_ABOUT, L"&About");

    AppendMenu(menu, MF_POPUP, (UINT_PTR) menuFile, L"&File");
    AppendMenu(menu, MF_POPUP, (UINT_PTR) menuView, L"&View");
    AppendMenu(menu, MF_POPUP, (UINT_PTR) menuSettings, L"&Settings");
    AppendMenu(menu, MF_POPUP, (UINT_PTR) menuHelp, L"&Help");
    SetMenu(hwnd, menu);

    int p=-1, baud=1050000;
    find_com_ports(hwnd);     
    if( wcsncmp(pCmdLine, L"COM", 3)==0 && wcslen(pCmdLine)<7 )
      p = _wtoi(pCmdLine+3);
    else 
      read_settings(&p, &baud);

    if( p>0 && p<256 )
      set_com_port(hwnd, p);

    set_baud_rate(hwnd, baud);

    if( g_com_port>0 || wcslen(pCmdLine)==0 )
      {
        DWORD id; 
        HANDLE h = CreateThread(0, 0, serial_thread, hwnd, 0, &id);
        CloseHandle(h);
      }
    else
      {
        peer = pCmdLine;
        server_socket = connect_socket(hwnd, peer);
        if( server_socket==INVALID_SOCKET )
          return 0;
        else if( WSAAsyncSelect(server_socket, hwnd, ID_SOCKET, FD_READ)!=0 )
          return 0;
        set_window_title(hwnd);
        RemoveMenu(menu, MF_BYPOSITION, 2);
      }

    for(int i=0; i<16; i++)
      {
        brushes_color[i]     = CreateSolidBrush(colors[i]);
        brushes_grayscale[i] = CreateSolidBrush(RGB(16*i,16*i,16*i));
      }

    ShowWindow(hwnd, SW_SHOW);
    update_frame(hwnd);    
    
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

    return 0;
}

#endif
