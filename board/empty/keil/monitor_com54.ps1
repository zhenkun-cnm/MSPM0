param(
    [string]$PortName = "COM54",
    [int]$BaudRate = 115200,
    [switch]$List,
    [switch]$Active,
    [string]$Type = ""
)

function Get-ComPortInfo {
    $result = @()
    
    Get-WmiObject Win32_PnPEntity | Where-Object { $_.Name -match 'COM\d+' } | ForEach-Object {
        $name = $_.Name
        $desc = $_.Description
        $pnpId = $_.PNPDeviceID
        $mfr = $_.Manufacturer
        
        # Extract COM number
        $port = ""
        if ($name -match '\((COM\d+)\)') {
            $port = $matches[1]
        } elseif ($name -match '(COM\d+)') {
            $port = $matches[1]
        }
        
        # Determine type
        $type = "SERIAL"
        $n = $name.ToLower()
        $d = $desc.ToLower()
        $p = $pnpId.ToLower()
        
        if ($n -match 'ch340' -or $d -match 'ch340' -or $p -match 'ch340') { $type = "CH340" }
        elseif ($n -match 'ftdi|ft232' -or $d -match 'ftdi|ft232' -or $p -match 'ftdi|ft232') { $type = "FTDI" }
        elseif ($n -match 'cp210|silicon' -or $d -match 'cp210|silicon' -or $p -match 'cp210|silicon') { $type = "CP210x" }
        elseif ($n -match 'pl2303|prolific' -or $d -match 'pl2303|prolific' -or $p -match 'pl2303|prolific') { $type = "PL2303" }
        elseif ($n -match 'xds110|xds' -or $d -match 'xds110|xds' -or $p -match 'xds110|xds') { $type = "XDS110" }
        elseif ($n -match 'arduino' -or $d -match 'arduino') { $type = "ARDUINO" }
        elseif ($n -match 'bluetooth' -or $d -match 'bluetooth' -or $n -match '\u84DD\u7259') { $type = "BLUETOOTH" }
        elseif ($n -match 'eltima|virtual' -or $d -match 'virtual') { $type = "VIRTUAL" }
        
        if ($port -ne "") {
            $result += [PSCustomObject]@{
                Port = $port
                Name = $name
                Type = $type
                Description = $desc
            }
        }
    }
    
    return ($result | Sort-Object { [int]($_.Port -replace 'COM', '') })
}

# List mode
if ($List) {
    $ports = Get-ComPortInfo
    
    if ($Active) {
        $ports = $ports | Where-Object { $_.Type -notin @("VIRTUAL", "BLUETOOTH") }
        Write-Host "`n[Active COM Ports (Physical)]"
    } else {
        Write-Host "`n[All Available COM Ports]"
    }
    
    Write-Host ("=" * 80)
    Write-Host ("{0,-8} {1,-12} {2,-50}" -f "Port", "Type", "Description")
    Write-Host ("-" * 80)
    
    foreach ($p in $ports) {
        Write-Host ("{0,-8} {1,-12} {2,-50}" -f $p.Port, $p.Type, $p.Name)
    }
    
    Write-Host ("-" * 80)
    if ($Active) {
        Write-Host "Hint: Use -List to show all ports`n"
    } else {
        Write-Host "Hint: Use -Type CH340 to auto-connect CH340 device`n"
    }
    exit 0
}

# Auto-detect by type
if ($Type -ne "" -and $PortName -eq "COM54") {
    $ports = Get-ComPortInfo
    $matched = $ports | Where-Object { $_.Type -eq $Type }
    
    if ($matched.Count -eq 0) {
        Write-Host "Error: No '$Type' port found!"
        exit 1
    }
    
    $PortName = $matched[0].Port
    Write-Host "Auto selected: $($matched[0].Port) ($($matched[0].Type))"
}

# Get port info
$allPorts = Get-ComPortInfo
$portInfo = $allPorts | Where-Object { $_.Port -eq $PortName }

# Check if COM port exists
$availablePorts = [System.IO.Ports.SerialPort]::GetPortNames()
if ($availablePorts -notcontains $PortName) {
    Write-Host "Error: $PortName not found!"
    Write-Host "Available ports: $($allPorts.Port -join ', ')"
    exit 1
}

$port = New-Object System.IO.Ports.SerialPort $PortName, $BaudRate, None, 8, One
$port.ReadTimeout = 5000
$port.WriteTimeout = 5000

try {
    $port.Open()
    $portType = if ($portInfo) { $portInfo.Type } else { "UNKNOWN" }
    Write-Host ""
    Write-Host ("=" * 54)
    Write-Host "  Port: $PortName"
    Write-Host "  Baud: $BaudRate"
    Write-Host "  Type: $portType"
    if ($portInfo) { Write-Host "  Desc: $($portInfo.Name)" }
    Write-Host ("=" * 54)
    Write-Host "  Listening... (Ctrl+C to exit)"
    Write-Host ""
} catch {
    Write-Host "Failed to open port: $($_.Exception.Message)"
    exit 1
}

while ($true) {
    try {
        $line = $port.ReadLine()
        Write-Host $line
    } catch [TimeoutException] {
        # Timeout is normal, continue
    } catch {
        $e = $_.Exception.Message
        Write-Host "Read error: $e"
        Start-Sleep -Milliseconds 100
    }
}

$port.Close()