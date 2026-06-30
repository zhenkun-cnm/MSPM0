# Refresh environment PATH first
$env:Path = [System.Environment]::GetEnvironmentVariable("Path","Machine") + ";" + [System.Environment]::GetEnvironmentVariable("Path","User")

# Try to locate npm
$npmPath = $null
$possiblePaths = @(
    "C:\Program Files\nodejs\npm.cmd",
    "C:\Program Files\nodejs\npm",
    "$env:LOCALAPPDATA\Programs\nodejs\npm.cmd",
    "$env:APPDATA\npm\npm.cmd"
)

foreach ($p in $possiblePaths) {
    if (Test-Path $p) {
        $npmPath = $p
        Write-Host "Found npm at: $npmPath"
        break
    }
}

if ($npmPath) {
    Write-Host "Installing Gemini CLI via npm..."
    & $npmPath install -g @google/gemini-cli
    Write-Host "Installation completed."
} else {
    Write-Host "npm not found. Searching for nodejs install..."
    # Search common locations
    Get-ChildItem -Path "C:\Program Files" -Filter "npm.cmd" -Recurse -ErrorAction SilentlyContinue -Depth 3 | ForEach-Object { Write-Host $_.FullName }
    Get-ChildItem -Path "C:\Program Files (x86)" -Filter "npm.cmd" -Recurse -ErrorAction SilentlyContinue -Depth 3 | ForEach-Object { Write-Host $_.FullName }
    # Try via where.exe in cmd
    cmd /c "where npm 2>&1"
}