$path = $env:APPDATA + "\Code\User\globalStorage"
Write-Host "Searching: $path"
Get-ChildItem -Path $path -Recurse | Where-Object { $_.Name -match "mcp|cline|claude" } | Select-Object FullName