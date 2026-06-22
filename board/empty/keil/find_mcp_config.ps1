$paths = @(
    "$env:APPDATA\Code\User\globalStorage",
    "$env:USERPROFILE\.config",
    "$env:LOCALAPPDATA\Code\User\globalStorage"
)

foreach ($p in $paths) {
    if (Test-Path $p) {
        Write-Host "Checking: $p"
        Get-ChildItem -Path $p -Filter "*.json" -Name | ForEach-Object {
            Write-Host "  Found: $_"
        }
    }
}