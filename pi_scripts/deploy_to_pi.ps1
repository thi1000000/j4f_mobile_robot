# Copy pi_scripts to Raspberry Pi and start web server
# Usage: .\deploy_to_pi.ps1 [-PiHost 192.168.2.242] [-PiUser pikii]
param(
    [string]$PiHost = "192.168.2.242",
    [string]$PiUser = "pikii"
)

$ErrorActionPreference = "Stop"
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RemoteDir = "/home/$PiUser/j4f_mobile_robot/pi_scripts"

Write-Host "Deploying to ${PiUser}@${PiHost} ..."

scp -r "$ScriptDir\*" "${PiUser}@${PiHost}:${RemoteDir}/"
ssh "${PiUser}@${PiHost}" "cd $RemoteDir && bash install_on_pi.sh"

Write-Host "Done. Open: http://${PiHost}:8080/"
