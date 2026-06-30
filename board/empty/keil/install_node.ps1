$nodeUrl = "https://nodejs.org/dist/v22.14.0/node-v22.14.0-x64.msi"
$installerPath = "$env:TEMP\nodejs_installer.msi"

Write-Host "Downloading Node.js from $nodeUrl ..."
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
Invoke-WebRequest -Uri $nodeUrl -OutFile $installerPath

Write-Host "Installing Node.js (this may take a minute)..."
Start-Process msiexec.exe -ArgumentList "/i `"$installerPath`" /quiet /norestart ADDLOCAL=ALL" -Wait -NoNewWindow

Write-Host "Node.js installation completed."

# Refresh PATH
$env:Path = [System.Environment]::GetEnvironmentVariable("Path","Machine") + ";" + [System.Environment]::GetEnvironmentVariable("Path","User")

Write-Host "Node version:"
node --version
Write-Host "npm version:"
npm --version