# Get serial port information
$ports = Get-WmiObject Win32_SerialPort | Select-Object DeviceID, Name, Description, Status, PNPDeviceID

Write-Host "============================================"
Write-Host "       Available Serial Ports"
Write-Host "============================================"
Write-Host ""

foreach ($port in $ports) {
    Write-Host ("COM Port: " + $port.DeviceID)
    Write-Host ("Name: " + $port.Name)
    Write-Host ("Desc: " + $port.Description)
    
    if ($port.PNPDeviceID -match 'BTHENUM') {
        Write-Host "Type: Bluetooth"
    } elseif ($port.PNPDeviceID -match 'USB') {
        Write-Host "Type: USB Serial"
    } else {
        Write-Host "Type: Other"
    }
    
    Write-Host ("Status: " + $port.Status)
    Write-Host ("PNP ID: " + $port.PNPDeviceID)
    Write-Host "--------------------------------------------"
    Write-Host ""
}

# Get ports from registry
Write-Host "============================================"
Write-Host "       All COM Ports from Registry"
Write-Host "============================================"
$comPorts = [System.IO.Ports.SerialPort]::GetPortNames()
foreach ($com in $comPorts) {
    Write-Host ("  " + $com)
}