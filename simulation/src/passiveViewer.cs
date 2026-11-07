using System;
using System.Runtime.InteropServices;
using System.Threading;
using Mujoco;

/// <summary>
/// Renders a live mjModel/mjData pair in an OS window on a dedicated thread, using MuJoCo's
/// own mjv/mjr scene and rendering API against a manually created WGL context (no GLFW needed).
/// </summary>
public sealed unsafe class PassiveViewer : IDisposable
{
	private readonly MujocoLib.mjModel_* _model;
	private readonly MujocoLib.mjData_* _data;
	private readonly object _dataLock;
	private readonly Thread _thread;
	private volatile bool _running = true;

	public PassiveViewer(MujocoLib.mjModel_* model, MujocoLib.mjData_* data, object dataLock, string title = "PhysicSim Viewer")
	{
		_model = model;
		_data = data;
		_dataLock = dataLock;
		_thread = new Thread(() => RunLoop(title)) { IsBackground = true, Name = "PassiveViewer" };
		_thread.Start();
	}

	public void Dispose()
	{
		_running = false;
		_thread.Join();
	}

	private void RunLoop(string title)
	{
		Win32Gl.CreateOpenGlWindow(title, 1200, 900, out IntPtr hwnd, out IntPtr hdc, out IntPtr hglrc);

		MujocoLib.mjvCamera_ cam;
		MujocoLib.mjvOption_ opt;
		MujocoLib.mjvPerturb_ pert;
		MujocoLib.mjvScene_ scn;
		MujocoLib.mjrContext_ con;

		MujocoLib.mjv_defaultCamera(&cam);
		MujocoLib.mjv_defaultFreeCamera(_model, &cam);
		cam.azimuth += 30; // rotate the view around the vertical (z) axis
		MujocoLib.mjv_defaultOption(&opt);
		MujocoLib.mjv_defaultPerturb(&pert);
		MujocoLib.mjv_defaultScene(&scn);
		MujocoLib.mjr_defaultContext(&con);

		MujocoLib.mjv_makeScene(_model, &scn, 2000);
		MujocoLib.mjr_makeContext(_model, &con, 150);

		try
		{
			while (_running)
			{
				if (!Win32Gl.PumpMessages())
				{
					break;
				}

				Win32Gl.GetClientSize(hwnd, out int width, out int height);
				if (width > 0 && height > 0)
				{
					lock (_dataLock)
					{
						MujocoLib.mjv_updateScene(_model, _data, &opt, &pert, &cam, (int)MujocoLib.mjtCatBit.mjCAT_ALL, &scn);
					}

					var viewport = new MujocoLib.mjrRect_ { left = 0, bottom = 0, width = width, height = height };
					MujocoLib.mjr_render(viewport, &scn, &con);
					Win32Gl.SwapBuffers(hdc);
				}

				Thread.Sleep(16);
			}
		}
		finally
		{
			MujocoLib.mjv_freeScene(&scn);
			MujocoLib.mjr_freeContext(&con);
			Win32Gl.DestroyOpenGlWindow(hwnd, hdc, hglrc);
		}
	}
}

/// <summary>
/// Minimal Win32 + WGL P/Invoke helpers to open a window with an OpenGL context, without GLFW.
/// </summary>
internal static class Win32Gl
{
	private const string ClassName = "PhysicSimViewerWindowClass";
	private static WndProcDelegate s_wndProcDelegate;
	private static bool s_classRegistered;

	private delegate IntPtr WndProcDelegate(IntPtr hWnd, uint msg, IntPtr wParam, IntPtr lParam);

	[StructLayout(LayoutKind.Sequential, CharSet = CharSet.Unicode)]
	private struct WNDCLASSEX
	{
		public uint cbSize;
		public uint style;
		public IntPtr lpfnWndProc;
		public int cbClsExtra;
		public int cbWndExtra;
		public IntPtr hInstance;
		public IntPtr hIcon;
		public IntPtr hCursor;
		public IntPtr hbrBackground;
		public string lpszMenuName;
		public string lpszClassName;
		public IntPtr hIconSm;
	}

	[StructLayout(LayoutKind.Sequential)]
	private struct MSG
	{
		public IntPtr hwnd;
		public uint message;
		public IntPtr wParam;
		public IntPtr lParam;
		public uint time;
		public int ptX;
		public int ptY;
	}

	[StructLayout(LayoutKind.Sequential)]
	private struct RECT { public int Left, Top, Right, Bottom; }

	[StructLayout(LayoutKind.Sequential)]
	private struct PIXELFORMATDESCRIPTOR
	{
		public ushort nSize;
		public ushort nVersion;
		public uint dwFlags;
		public byte iPixelType;
		public byte cColorBits;
		public byte cRedBits, cRedShift, cGreenBits, cGreenShift, cBlueBits, cBlueShift, cAlphaBits, cAlphaShift;
		public byte cAccumBits, cAccumRedBits, cAccumGreenBits, cAccumBlueBits, cAccumAlphaBits;
		public byte cDepthBits, cStencilBits, cAuxBuffers;
		public byte iLayerType;
		public byte bReserved;
		public uint dwLayerMask, dwVisibleMask, dwDamageMask;
	}

	private const uint CS_OWNDC = 0x0020;
	private const uint WS_OVERLAPPEDWINDOW = 0x00CF0000;
	private const uint WS_VISIBLE = 0x10000000;
	private const uint SW_SHOW = 5;
	private const uint WM_DESTROY = 0x0002;
	private const uint WM_CLOSE = 0x0010;
	private const uint WM_QUIT = 0x0012;
	private const uint PM_REMOVE = 0x0001;
	private const int CW_USEDEFAULT = unchecked((int)0x80000000);

	private const byte PFD_TYPE_RGBA = 0;
	private const byte PFD_MAIN_PLANE = 0;
	private const uint PFD_DOUBLEBUFFER = 0x00000001;
	private const uint PFD_DRAW_TO_WINDOW = 0x00000004;
	private const uint PFD_SUPPORT_OPENGL = 0x00000020;

	[DllImport("user32.dll", CharSet = CharSet.Unicode)]
	private static extern ushort RegisterClassEx(ref WNDCLASSEX lpwcx);

	[DllImport("user32.dll", CharSet = CharSet.Unicode)]
	private static extern IntPtr CreateWindowEx(uint dwExStyle, string lpClassName, string lpWindowName, uint dwStyle,
		int x, int y, int nWidth, int nHeight, IntPtr hWndParent, IntPtr hMenu, IntPtr hInstance, IntPtr lpParam);

	[DllImport("user32.dll")]
	private static extern IntPtr DefWindowProc(IntPtr hWnd, uint msg, IntPtr wParam, IntPtr lParam);

	[DllImport("user32.dll")]
	private static extern bool ShowWindow(IntPtr hWnd, uint nCmdShow);

	[DllImport("user32.dll")]
	private static extern bool UpdateWindow(IntPtr hWnd);

	[DllImport("user32.dll")]
	private static extern bool PeekMessage(out MSG lpMsg, IntPtr hWnd, uint wMsgFilterMin, uint wMsgFilterMax, uint wRemoveMsg);

	[DllImport("user32.dll")]
	private static extern bool TranslateMessage(ref MSG lpMsg);

	[DllImport("user32.dll")]
	private static extern IntPtr DispatchMessage(ref MSG lpMsg);

	[DllImport("user32.dll")]
	private static extern void PostQuitMessage(int nExitCode);

	[DllImport("user32.dll")]
	private static extern bool DestroyWindow(IntPtr hWnd);

	[DllImport("user32.dll")]
	private static extern IntPtr GetDC(IntPtr hWnd);

	[DllImport("user32.dll")]
	private static extern int ReleaseDC(IntPtr hWnd, IntPtr hDC);

	[DllImport("user32.dll")]
	private static extern bool GetClientRect(IntPtr hWnd, out RECT lpRect);

	[DllImport("kernel32.dll")]
	private static extern IntPtr GetModuleHandle(string lpModuleName);

	[DllImport("gdi32.dll")]
	private static extern int ChoosePixelFormat(IntPtr hdc, ref PIXELFORMATDESCRIPTOR ppfd);

	[DllImport("gdi32.dll")]
	private static extern bool SetPixelFormat(IntPtr hdc, int format, ref PIXELFORMATDESCRIPTOR ppfd);

	[DllImport("gdi32.dll")]
	public static extern bool SwapBuffers(IntPtr hdc);

	[DllImport("opengl32.dll")]
	private static extern IntPtr wglCreateContext(IntPtr hdc);

	[DllImport("opengl32.dll")]
	private static extern bool wglMakeCurrent(IntPtr hdc, IntPtr hglrc);

	[DllImport("opengl32.dll")]
	private static extern bool wglDeleteContext(IntPtr hglrc);

	public static void CreateOpenGlWindow(string title, int width, int height, out IntPtr hwnd, out IntPtr hdc, out IntPtr hglrc)
	{
		IntPtr hInstance = GetModuleHandle(null);

		if (!s_classRegistered)
		{
			s_wndProcDelegate = WndProc;
			var wc = new WNDCLASSEX
			{
				cbSize = (uint)Marshal.SizeOf(typeof(WNDCLASSEX)),
				style = CS_OWNDC,
				lpfnWndProc = Marshal.GetFunctionPointerForDelegate(s_wndProcDelegate),
				hInstance = hInstance,
				lpszClassName = ClassName,
			};
			if (RegisterClassEx(ref wc) == 0)
			{
				throw new InvalidOperationException("Failed to register the viewer window class.");
			}
			s_classRegistered = true;
		}

		hwnd = CreateWindowEx(0, ClassName, title, WS_OVERLAPPEDWINDOW | WS_VISIBLE,
			CW_USEDEFAULT, CW_USEDEFAULT, width, height, IntPtr.Zero, IntPtr.Zero, hInstance, IntPtr.Zero);
		if (hwnd == IntPtr.Zero)
		{
			throw new InvalidOperationException("Failed to create the viewer window.");
		}

		ShowWindow(hwnd, SW_SHOW);
		UpdateWindow(hwnd);

		hdc = GetDC(hwnd);

		var pfd = new PIXELFORMATDESCRIPTOR
		{
			nSize = (ushort)Marshal.SizeOf(typeof(PIXELFORMATDESCRIPTOR)),
			nVersion = 1,
			dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,
			iPixelType = PFD_TYPE_RGBA,
			cColorBits = 32,
			cDepthBits = 24,
			cStencilBits = 8,
			iLayerType = PFD_MAIN_PLANE,
		};

		int pixelFormat = ChoosePixelFormat(hdc, ref pfd);
		if (pixelFormat == 0 || !SetPixelFormat(hdc, pixelFormat, ref pfd))
		{
			throw new InvalidOperationException("Failed to set an OpenGL-compatible pixel format.");
		}

		hglrc = wglCreateContext(hdc);
		if (hglrc == IntPtr.Zero || !wglMakeCurrent(hdc, hglrc))
		{
			throw new InvalidOperationException("Failed to create the OpenGL context.");
		}
	}

	public static void DestroyOpenGlWindow(IntPtr hwnd, IntPtr hdc, IntPtr hglrc)
	{
		wglMakeCurrent(IntPtr.Zero, IntPtr.Zero);
		wglDeleteContext(hglrc);
		ReleaseDC(hwnd, hdc);
		DestroyWindow(hwnd);
	}

	public static bool PumpMessages()
	{
		while (PeekMessage(out MSG msg, IntPtr.Zero, 0, 0, PM_REMOVE))
		{
			if (msg.message == WM_QUIT)
			{
				return false;
			}
			TranslateMessage(ref msg);
			DispatchMessage(ref msg);
		}
		return true;
	}

	public static void GetClientSize(IntPtr hwnd, out int width, out int height)
	{
		GetClientRect(hwnd, out RECT rect);
		width = rect.Right - rect.Left;
		height = rect.Bottom - rect.Top;
	}

	private static IntPtr WndProc(IntPtr hWnd, uint msg, IntPtr wParam, IntPtr lParam)
	{
		if (msg == WM_CLOSE || msg == WM_DESTROY)
		{
			PostQuitMessage(0);
			return IntPtr.Zero;
		}
		return DefWindowProc(hWnd, msg, wParam, lParam);
	}
}
