# ==============================================================================
# Windows to RPi5 Deployment Script
# Run this PowerShell script from your Windows dev machine to deploy files.
# ==============================================================================
# Target configuration variables (will be loaded from .env)
$IP = ""
$USER = ""
$PASSWORD = ""

# Load environment variables from .env if present
if (Test-Path ".env") {
    Get-Content ".env" | ForEach-Object {
        $line = $_.Trim()
        if ($line -and -not $line.StartsWith("#")) {
            $parts = $line.Split("=", 2)
            if ($parts.Length -eq 2) {
                $key = $parts[0].Trim()
                $val = $parts[1].Trim()
                if ($key -eq "RPI_IP") { $IP = $val }
                elseif ($key -eq "RPI_USER") { $USER = $val }
                elseif ($key -eq "RPI_PASSWORD") { $PASSWORD = $val }
            }
        }
    }
}

# Verify that configuration has been configured
if ([string]::IsNullOrWhiteSpace($IP) -or [string]::IsNullOrWhiteSpace($USER) -or [string]::IsNullOrWhiteSpace($PASSWORD) -or $IP -eq "<your_rpi5_ip_address>" -or $PASSWORD -eq "<your_rpi5_password>") {
    Write-Warning "Please configure your target RPi5 details (RPI_IP, RPI_USER, RPI_PASSWORD) in the .env file before running."
    exit 1
}

$DEST_DIR = "/home/$USER/rover-encoder"

Write-Host "=== Packaging Rover project files ===" -ForegroundColor Cyan
if (Test-Path "rover.tar.gz") {
    Remove-Item "rover.tar.gz"
}

# Create a tarball excluding node_modules, .git, and platformio build folder
tar --exclude="node_modules" --exclude=".git" --exclude="maker_esp32_pro/.pio" -czf rover.tar.gz * .env*

if (-not (Test-Path "rover.tar.gz")) {
    Write-Error "Failed to create rover.tar.gz archive."
    exit 1
}

Write-Host "=== Transferring archive to RPi5 ($IP) ===" -ForegroundColor Cyan
Write-Host "Password is: $PASSWORD (if prompted)" -ForegroundColor Yellow
scp rover.tar.gz "${USER}@${IP}:/home/${USER}/"

if ($LASTEXITCODE -ne 0) {
    Write-Error "Failed to transfer files to RPi5."
    exit 1
}

Write-Host "=== Unpacking and running setup on RPi5 ===" -ForegroundColor Cyan
Write-Host "Enter '$PASSWORD' when prompted for SSH password or sudo password." -ForegroundColor Yellow

# SSH into RPi5, unpack archive, make setup.sh executable and run it
ssh -t "${USER}@${IP}" "mkdir -p ${DEST_DIR} && tar -xzf ~/rover.tar.gz -C ${DEST_DIR} && cd ${DEST_DIR} && chmod +x rpi5/setup.sh && sudo ./rpi5/setup.sh"

# Clean up local archive
Remove-Item "rover.tar.gz"

Write-Host "=== Deployment process completed ===" -ForegroundColor Green
