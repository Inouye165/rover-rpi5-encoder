# ==============================================================================
# Windows to RPi5 Deployment Script
# Run this PowerShell script from your Windows dev machine to deploy files.
# ==============================================================================

$IP = "<your_rpi5_ip_address>"
$USER = "ron"
$DEST_DIR = "/home/$USER/yahboom-encoder"

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
Write-Host "Password is: <your_rpi5_password> (if prompted)" -ForegroundColor Yellow
scp rover.tar.gz "${USER}@${IP}:/home/${USER}/"

if ($LASTEXITCODE -ne 0) {
    Write-Error "Failed to transfer files to RPi5."
    exit 1
}

Write-Host "=== Unpacking and running setup on RPi5 ===" -ForegroundColor Cyan
Write-Host "Enter '<your_rpi5_password>' when prompted for SSH password or sudo password." -ForegroundColor Yellow

# SSH into RPi5, unpack archive, make setup.sh executable and run it
ssh -t "${USER}@${IP}" "mkdir -p ${DEST_DIR} && tar -xzf ~/rover.tar.gz -C ${DEST_DIR} && cd ${DEST_DIR} && chmod +x rpi5/setup.sh && sudo ./rpi5/setup.sh"

# Clean up local archive
Remove-Item "rover.tar.gz"

Write-Host "=== Deployment process completed ===" -ForegroundColor Green
