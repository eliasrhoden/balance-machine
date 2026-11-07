using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Runtime.InteropServices;
using System.Xml.Linq;

public enum FmuVariableType { Real, Integer, Boolean, String }

public readonly struct FmuVariableInfo
{
    public string Name { get; }
    public uint ValueReference { get; }
    public FmuVariableType Type { get; }
    public string Causality { get; }

    public FmuVariableInfo(string name, uint valueReference, FmuVariableType type, string causality)
    {
        Name = name;
        ValueReference = valueReference;
        Type = type;
        Causality = causality;
    }
}

internal static class NativeMethods
{
    [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
    public static extern IntPtr LoadLibrary(string dllPath);

    [DllImport("kernel32.dll", SetLastError = true)]
    public static extern IntPtr GetProcAddress(IntPtr hModule, string procName);

    [DllImport("kernel32.dll", SetLastError = true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    public static extern bool FreeLibrary(IntPtr hModule);
}

/// <summary>
/// Generic FMI 2.0 Co-Simulation client. Loads a .fmu (a zip archive), reads modelDescription.xml
/// and calls the platform-specific native binary directly via P/Invoke.
/// </summary>
public sealed class FmuCoSimulation : IDisposable
{
    private enum Fmi2Status { OK = 0, Warning = 1, Discard = 2, Error = 3, Fatal = 4, Pending = 5 }

    [StructLayout(LayoutKind.Sequential)]
    private struct Fmi2CallbackFunctions
    {
        public IntPtr logger;
        public IntPtr allocateMemory;
        public IntPtr freeMemory;
        public IntPtr stepFinished;
        public IntPtr componentEnvironment;
    }

    [UnmanagedFunctionPointer(CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    private delegate void CallbackLoggerDelegate(IntPtr componentEnvironment, string instanceName, int status, string category, string message);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate IntPtr CallbackAllocateMemoryDelegate(UIntPtr nobj, UIntPtr size);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate void CallbackFreeMemoryDelegate(IntPtr obj);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    private delegate IntPtr Fmi2InstantiateDelegate(string instanceName, int fmuType, string fmuGUID, string fmuResourceLocation, IntPtr functions, int visible, int loggingOn);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate void Fmi2FreeInstanceDelegate(IntPtr component);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate int Fmi2SetupExperimentDelegate(IntPtr component, int toleranceDefined, double tolerance, double startTime, int stopTimeDefined, double stopTime);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate int Fmi2SimpleCallDelegate(IntPtr component);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate int Fmi2DoStepDelegate(IntPtr component, double currentCommunicationPoint, double communicationStepSize, int noSetFMUStatePriorToCurrentPoint);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate int Fmi2GetRealDelegate(IntPtr component, uint[] vr, UIntPtr nvr, [Out] double[] value);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate int Fmi2SetRealDelegate(IntPtr component, uint[] vr, UIntPtr nvr, double[] value);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate int Fmi2GetIntegerDelegate(IntPtr component, uint[] vr, UIntPtr nvr, [Out] int[] value);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate int Fmi2SetIntegerDelegate(IntPtr component, uint[] vr, UIntPtr nvr, int[] value);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate int Fmi2GetBooleanDelegate(IntPtr component, uint[] vr, UIntPtr nvr, [Out] int[] value);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate int Fmi2SetBooleanDelegate(IntPtr component, uint[] vr, UIntPtr nvr, int[] value);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate int Fmi2GetStringDelegate(IntPtr component, uint[] vr, UIntPtr nvr, [Out] IntPtr[] value);
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    private delegate int Fmi2SetStringDelegate(IntPtr component, uint[] vr, UIntPtr nvr, IntPtr[] value);

    private readonly string _extractDir;
    private readonly IntPtr _libraryHandle;
    private readonly IntPtr _component;
    private IntPtr _callbackFunctionsMemory;
    private readonly Dictionary<string, FmuVariableInfo> _variables;

    // Kept alive so the native side can safely hold onto the function pointers.
    private readonly CallbackLoggerDelegate _loggerCallback;
    private readonly CallbackAllocateMemoryDelegate _allocateMemoryCallback;
    private readonly CallbackFreeMemoryDelegate _freeMemoryCallback;

    private readonly Fmi2FreeInstanceDelegate _fmi2FreeInstance;
    private readonly Fmi2SetupExperimentDelegate _fmi2SetupExperiment;
    private readonly Fmi2SimpleCallDelegate _fmi2EnterInitializationMode;
    private readonly Fmi2SimpleCallDelegate _fmi2ExitInitializationMode;
    private readonly Fmi2SimpleCallDelegate _fmi2Terminate;
    private readonly Fmi2SimpleCallDelegate _fmi2Reset;
    private readonly Fmi2DoStepDelegate _fmi2DoStep;
    private readonly Fmi2GetRealDelegate _fmi2GetReal;
    private readonly Fmi2SetRealDelegate _fmi2SetReal;
    private readonly Fmi2GetIntegerDelegate _fmi2GetInteger;
    private readonly Fmi2SetIntegerDelegate _fmi2SetInteger;
    private readonly Fmi2GetBooleanDelegate _fmi2GetBoolean;
    private readonly Fmi2SetBooleanDelegate _fmi2SetBoolean;
    private readonly Fmi2GetStringDelegate _fmi2GetString;
    private readonly Fmi2SetStringDelegate _fmi2SetString;

    private bool _disposed;
    private bool _terminated;

    public string ModelName { get; }
    public string ModelIdentifier { get; }
    public string Guid { get; }
    public double DefaultStartTime { get; }
    public double DefaultStepSize { get; }
    public IReadOnlyCollection<string> VariableNames => _variables.Keys.ToArray();

    public FmuCoSimulation(string fmuPath, string instanceName = "instance1", bool loggingEnabled = false)
    {
        if (!File.Exists(fmuPath))
        {
            throw new FileNotFoundException("FMU file not found.", fmuPath);
        }

        _extractDir = Path.Combine(Path.GetTempPath(), "fmu_" + System.Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(_extractDir);
        ZipFile.ExtractToDirectory(fmuPath, _extractDir);

        ParseModelDescription(Path.Combine(_extractDir, "modelDescription.xml"),
            out var guid, out var modelIdentifier, out var modelName,
            out var variables, out var startTime, out var stepSize);

        Guid = guid;
        ModelIdentifier = modelIdentifier;
        ModelName = modelName;
        _variables = variables;
        DefaultStartTime = startTime;
        DefaultStepSize = stepSize;

        string platformDir = Environment.Is64BitProcess ? "win64" : "win32";
        string dllPath = Path.Combine(_extractDir, "binaries", platformDir, modelIdentifier + ".dll");
        if (!File.Exists(dllPath))
        {
            throw new PlatformNotSupportedException($"No FMU binary found for platform '{platformDir}' at '{dllPath}'.");
        }

        _libraryHandle = NativeMethods.LoadLibrary(dllPath);
        if (_libraryHandle == IntPtr.Zero)
        {
            throw new InvalidOperationException($"Failed to load FMU library '{dllPath}' (Win32 error {Marshal.GetLastWin32Error()}).");
        }

        var instantiate = GetDelegate<Fmi2InstantiateDelegate>("fmi2Instantiate");
        _fmi2FreeInstance = GetDelegate<Fmi2FreeInstanceDelegate>("fmi2FreeInstance");
        _fmi2SetupExperiment = GetDelegate<Fmi2SetupExperimentDelegate>("fmi2SetupExperiment");
        _fmi2EnterInitializationMode = GetDelegate<Fmi2SimpleCallDelegate>("fmi2EnterInitializationMode");
        _fmi2ExitInitializationMode = GetDelegate<Fmi2SimpleCallDelegate>("fmi2ExitInitializationMode");
        _fmi2Terminate = GetDelegate<Fmi2SimpleCallDelegate>("fmi2Terminate");
        _fmi2Reset = GetDelegate<Fmi2SimpleCallDelegate>("fmi2Reset");
        _fmi2DoStep = GetDelegate<Fmi2DoStepDelegate>("fmi2DoStep");
        _fmi2GetReal = GetDelegate<Fmi2GetRealDelegate>("fmi2GetReal");
        _fmi2SetReal = GetDelegate<Fmi2SetRealDelegate>("fmi2SetReal");
        _fmi2GetInteger = GetDelegate<Fmi2GetIntegerDelegate>("fmi2GetInteger");
        _fmi2SetInteger = GetDelegate<Fmi2SetIntegerDelegate>("fmi2SetInteger");
        _fmi2GetBoolean = GetDelegate<Fmi2GetBooleanDelegate>("fmi2GetBoolean");
        _fmi2SetBoolean = GetDelegate<Fmi2SetBooleanDelegate>("fmi2SetBoolean");
        _fmi2GetString = GetDelegate<Fmi2GetStringDelegate>("fmi2GetString");
        _fmi2SetString = GetDelegate<Fmi2SetStringDelegate>("fmi2SetString");

        _loggerCallback = LoggerCallback;
        _allocateMemoryCallback = AllocateMemoryCallback;
        _freeMemoryCallback = FreeMemoryCallback;

        var callbacks = new Fmi2CallbackFunctions
        {
            logger = Marshal.GetFunctionPointerForDelegate(_loggerCallback),
            allocateMemory = Marshal.GetFunctionPointerForDelegate(_allocateMemoryCallback),
            freeMemory = Marshal.GetFunctionPointerForDelegate(_freeMemoryCallback),
            stepFinished = IntPtr.Zero,
            componentEnvironment = IntPtr.Zero
        };

        string resourceLocation = new Uri(Path.Combine(_extractDir, "resources") + Path.DirectorySeparatorChar).AbsoluteUri;

        _callbackFunctionsMemory = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(Fmi2CallbackFunctions)));
        Marshal.StructureToPtr(callbacks, _callbackFunctionsMemory, false);

        _component = instantiate(instanceName, /* fmi2CoSimulation */ 1, guid, resourceLocation, _callbackFunctionsMemory, 0, loggingEnabled ? 1 : 0);
        if (_component == IntPtr.Zero)
        {
            throw new InvalidOperationException("fmi2Instantiate failed - the FMU refused to create an instance.");
        }
    }

    public void SetupExperiment(double startTime = 0, double? stopTime = null, double tolerance = 0)
    {
        int rc = _fmi2SetupExperiment(_component, tolerance > 0 ? 1 : 0, tolerance, startTime, stopTime.HasValue ? 1 : 0, stopTime ?? 0);
        ThrowOnError(rc, nameof(SetupExperiment));
    }

    public void EnterInitializationMode() => ThrowOnError(_fmi2EnterInitializationMode(_component), nameof(EnterInitializationMode));

    public void ExitInitializationMode() => ThrowOnError(_fmi2ExitInitializationMode(_component), nameof(ExitInitializationMode));

    public void Reset() => ThrowOnError(_fmi2Reset(_component), nameof(Reset));

    public void Terminate()
    {
        if (_terminated) return;
        ThrowOnError(_fmi2Terminate(_component), nameof(Terminate));
        _terminated = true;
    }

    public void DoStep(double currentTime, double stepSize, bool noSetFmuStatePriorToCurrentPoint = true)
    {
        int rc = _fmi2DoStep(_component, currentTime, stepSize, noSetFmuStatePriorToCurrentPoint ? 1 : 0);
        ThrowOnError(rc, nameof(DoStep));
    }

    public FmuVariableInfo GetVariableInfo(string name) =>
        _variables.TryGetValue(name, out var info) ? info : throw new KeyNotFoundException($"Unknown FMU variable '{name}'.");

    public double GetReal(string name) => GetReal(RequireVariable(name, FmuVariableType.Real).ValueReference);
    public double GetReal(uint valueReference)
    {
        var values = new double[1];
        ThrowOnError(_fmi2GetReal(_component, new[] { valueReference }, (UIntPtr)1u, values), nameof(GetReal));
        return values[0];
    }

    public void SetReal(string name, double value) => SetReal(RequireVariable(name, FmuVariableType.Real).ValueReference, value);
    public void SetReal(uint valueReference, double value) =>
        ThrowOnError(_fmi2SetReal(_component, new[] { valueReference }, (UIntPtr)1u, new[] { value }), nameof(SetReal));

    public int GetInteger(string name) => GetInteger(RequireVariable(name, FmuVariableType.Integer).ValueReference);
    public int GetInteger(uint valueReference)
    {
        var values = new int[1];
        ThrowOnError(_fmi2GetInteger(_component, new[] { valueReference }, (UIntPtr)1u, values), nameof(GetInteger));
        return values[0];
    }

    public void SetInteger(string name, int value) => SetInteger(RequireVariable(name, FmuVariableType.Integer).ValueReference, value);
    public void SetInteger(uint valueReference, int value) =>
        ThrowOnError(_fmi2SetInteger(_component, new[] { valueReference }, (UIntPtr)1u, new[] { value }), nameof(SetInteger));

    public bool GetBoolean(string name) => GetBoolean(RequireVariable(name, FmuVariableType.Boolean).ValueReference);
    public bool GetBoolean(uint valueReference)
    {
        var values = new int[1];
        ThrowOnError(_fmi2GetBoolean(_component, new[] { valueReference }, (UIntPtr)1u, values), nameof(GetBoolean));
        return values[0] != 0;
    }

    public void SetBoolean(string name, bool value) => SetBoolean(RequireVariable(name, FmuVariableType.Boolean).ValueReference, value);
    public void SetBoolean(uint valueReference, bool value) =>
        ThrowOnError(_fmi2SetBoolean(_component, new[] { valueReference }, (UIntPtr)1u, new[] { value ? 1 : 0 }), nameof(SetBoolean));

    public string GetString(string name) => GetString(RequireVariable(name, FmuVariableType.String).ValueReference);
    public string GetString(uint valueReference)
    {
        var values = new IntPtr[1];
        ThrowOnError(_fmi2GetString(_component, new[] { valueReference }, (UIntPtr)1u, values), nameof(GetString));
        return values[0] == IntPtr.Zero ? null : Marshal.PtrToStringAnsi(values[0]);
    }

    public void SetString(string name, string value) => SetString(RequireVariable(name, FmuVariableType.String).ValueReference, value);
    public void SetString(uint valueReference, string value)
    {
        IntPtr ptr = Marshal.StringToHGlobalAnsi(value ?? string.Empty);
        try
        {
            ThrowOnError(_fmi2SetString(_component, new[] { valueReference }, (UIntPtr)1u, new[] { ptr }), nameof(SetString));
        }
        finally
        {
            Marshal.FreeHGlobal(ptr);
        }
    }

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;

        if (!_terminated && _component != IntPtr.Zero && _fmi2Terminate != null)
        {
            try { _fmi2Terminate(_component); } catch { /* best effort */ }
        }

        if (_component != IntPtr.Zero && _fmi2FreeInstance != null)
        {
            _fmi2FreeInstance(_component);
        }

        if (_callbackFunctionsMemory != IntPtr.Zero)
        {
            Marshal.FreeHGlobal(_callbackFunctionsMemory);
            _callbackFunctionsMemory = IntPtr.Zero;
        }

        if (_libraryHandle != IntPtr.Zero)
        {
            NativeMethods.FreeLibrary(_libraryHandle);
        }

        try
        {
            if (Directory.Exists(_extractDir)) Directory.Delete(_extractDir, true);
        }
        catch { /* best effort cleanup */ }
    }

    private FmuVariableInfo RequireVariable(string name, FmuVariableType expectedType)
    {
        var info = GetVariableInfo(name);
        if (info.Type != expectedType)
        {
            throw new InvalidOperationException($"Variable '{name}' is of type {info.Type}, not {expectedType}.");
        }
        return info;
    }

    private TDelegate GetDelegate<TDelegate>(string name) where TDelegate : class
    {
        IntPtr proc = NativeMethods.GetProcAddress(_libraryHandle, name);
        if (proc == IntPtr.Zero)
        {
            throw new EntryPointNotFoundException($"FMU library does not export function '{name}'.");
        }
        return Marshal.GetDelegateForFunctionPointer<TDelegate>(proc);
    }

    private static void ThrowOnError(int status, string operation)
    {
        if (status == (int)Fmi2Status.Error || status == (int)Fmi2Status.Fatal)
        {
            throw new InvalidOperationException($"FMU call '{operation}' failed with status {(Fmi2Status)status}.");
        }
    }

    private static void LoggerCallback(IntPtr componentEnvironment, string instanceName, int status, string category, string message)
    {
        // TEMP DIAGNOSTIC: no-op to check if the logger callback itself is what crashes.
    }

    private static IntPtr AllocateMemoryCallback(UIntPtr nobj, UIntPtr size)
    {
        long total = (long)((ulong)nobj * (ulong)size);
        if (total <= 0) return IntPtr.Zero;
        IntPtr ptr = Marshal.AllocHGlobal(new IntPtr(total));
        for (long i = 0; i < total; i++) Marshal.WriteByte(ptr, (int)i, 0);
        return ptr;
    }

    private static void FreeMemoryCallback(IntPtr obj)
    {
        if (obj != IntPtr.Zero) Marshal.FreeHGlobal(obj);
    }

    private static void ParseModelDescription(string path, out string guid, out string modelIdentifier, out string modelName,
        out Dictionary<string, FmuVariableInfo> variables, out double startTime, out double stepSize)
    {
        var doc = XDocument.Load(path);
        var root = doc.Root;
        if (root == null || root.Name.LocalName != "fmiModelDescription")
        {
            throw new InvalidDataException("Invalid FMU: modelDescription.xml is missing the root element.");
        }

        string fmiVersion = (string)root.Attribute("fmiVersion");
        if (fmiVersion == null || !fmiVersion.StartsWith("2.0"))
        {
            throw new NotSupportedException($"Only FMI 2.0 FMUs are supported (found '{fmiVersion}').");
        }

        var coSimulation = root.Element("CoSimulation");
        if (coSimulation == null)
        {
            throw new NotSupportedException("The FMU does not implement Co-Simulation.");
        }

        guid = (string)root.Attribute("guid");
        modelName = (string)root.Attribute("modelName");
        modelIdentifier = (string)coSimulation.Attribute("modelIdentifier");

        var defaultExperiment = root.Element("DefaultExperiment");
        startTime = (double?)defaultExperiment?.Attribute("startTime") ?? 0;
        stepSize = (double?)defaultExperiment?.Attribute("stepSize") ?? 0;

        variables = new Dictionary<string, FmuVariableInfo>(StringComparer.Ordinal);
        var modelVariables = root.Element("ModelVariables");
        if (modelVariables == null) return;

        foreach (var scalarVariable in modelVariables.Elements("ScalarVariable"))
        {
            string name = (string)scalarVariable.Attribute("name");
            uint valueReference = (uint)(int)scalarVariable.Attribute("valueReference");
            string causality = (string)scalarVariable.Attribute("causality") ?? "local";

            var typeElement = scalarVariable.Elements().FirstOrDefault(e =>
                e.Name.LocalName == "Real" || e.Name.LocalName == "Integer" ||
                e.Name.LocalName == "Boolean" || e.Name.LocalName == "String");

            FmuVariableType type;
            switch (typeElement?.Name.LocalName)
            {
                case "Integer": type = FmuVariableType.Integer; break;
                case "Boolean": type = FmuVariableType.Boolean; break;
                case "String": type = FmuVariableType.String; break;
                default: type = FmuVariableType.Real; break;
            }

            variables[name] = new FmuVariableInfo(name, valueReference, type, causality);
        }
    }
}

public readonly struct FmuParameterValue
{
    public string Name { get; }
    public double Value { get; }

    public FmuParameterValue(string name, double value)
    {
        Name = name;
        Value = value;
    }
}

/// <summary>Loads FMU "parameter" (causality=parameter) values from an XML config, e.g. DriveSimDesigner_SERVO.Parameters.xml.</summary>
public sealed class FmuParameterSet
{
    public IReadOnlyList<FmuParameterValue> Parameters { get; }

    private FmuParameterSet(List<FmuParameterValue> parameters)
    {
        Parameters = parameters;
    }

    public static FmuParameterSet Load(string path)
    {
        var doc = XDocument.Load(path);
        var root = doc.Root ?? throw new InvalidDataException($"'{path}' has no root element.");

        var parameters = new List<FmuParameterValue>();
        foreach (var element in root.Elements("Parameter"))
        {
            string name = (string)element.Attribute("name") ?? throw new InvalidDataException("A <Parameter> element is missing its 'name' attribute.");
            double value = (double)element.Attribute("value");
            parameters.Add(new FmuParameterValue(name, value));
        }
        return new FmuParameterSet(parameters);
    }

    // Call between EnterInitializationMode() and ExitInitializationMode() per the FMI2 spec for "fixed" variability parameters.
    public void ApplyTo(FmuCoSimulation fmu)
    {
        foreach (var parameter in Parameters)
        {
            // TEMP DIAGNOSTIC: flush before the native call so the last printed name survives a native crash.
            Console.WriteLine($"[FmuParameterSet] SetReal {parameter.Name} = {parameter.Value}");
            Console.Out.Flush();
            fmu.SetReal(parameter.Name, parameter.Value);
        }
    }
}
