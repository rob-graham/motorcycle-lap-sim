# Windows repo-root CLI launcher

For the development checkout on Windows, `motorcycle-lap-sim.cmd` provides a repo-root launcher for the installed `motorcycle-lap-sim` console executable.

From PowerShell in the repository root:

```powershell
.\motorcycle-lap-sim.cmd --help
.\motorcycle-lap-sim.cmd export runoff "C:\path\to\reduced_51_final_controls.csv"
```

The launcher looks first for:

```text
.venv-numba\Scripts\motorcycle-lap-sim.exe
```

and then for:

```text
.venv\Scripts\motorcycle-lap-sim.exe
```

It forwards all supplied arguments unchanged and returns the executable's exit status. It does not activate a virtual environment or alter simulation/export behaviour.
