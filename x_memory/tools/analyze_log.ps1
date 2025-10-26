# Computes metrics from AirSim hover_bias 10 Hz summary logs
# Usage: ./analyze_log.ps1 -Path "D:\open\airsim\x_memory\LOGS\20251026_161648.LOG"
param(
    [Parameter(Mandatory=$false)][string]$Path,
    [Parameter(Mandatory=$false)][string]$Dir,
    [Parameter(Mandatory=$false)][string]$Scenario = 'hover_bias',
    [Parameter(Mandatory=$false)][int]$StartSec = 30,
    [Parameter(Mandatory=$false)][int]$EndSec = 120,
    [Parameter(Mandatory=$false)][int]$Hz = 10
)

# Resolve input file
if (-not $Path -and $Dir) {
    $latest = Get-ChildItem -LiteralPath $Dir -File -Filter *.LOG | Sort-Object LastWriteTime -Descending | Select-Object -First 1
    if ($null -eq $latest) { Write-Error "No .LOG files in $Dir"; exit 1 }
    $Path = $latest.FullName
}
if (-not $Path) { Write-Error "Specify -Path or -Dir"; exit 1 }
if (!(Test-Path -LiteralPath $Path)) { Write-Error "File not found: $Path"; exit 1 }

$all = Get-Content -LiteralPath $Path
$hover = @()
foreach ($l in $all) {
    if ($l -like "$Scenario*") { $hover += $l }
}
if ($hover.Count -eq 0) { Write-Error "No hover_bias lines"; exit 1 }

# Take window StartSec..EndSec at given Hz if available
$skip = [math]::Max(0, $StartSec * $Hz)
$take = if ($EndSec -gt 0 -and $EndSec -gt $StartSec) { ($EndSec - $StartSec) * $Hz } else { 0 }
if ($hover.Count -le $skip) { Write-Error "Not enough lines for start=$StartSec s"; exit 1 }
if ($take -gt 0) {
    $last = [math]::Min($hover.Count-1, $skip + $take - 1)
    $win = $hover[$skip..$last]
} else {
    $win = $hover[$skip..($hover.Count-1)]
}

$rxYaw = [regex]'yaw\[rate,cy\]=\s*([\-0-9\.]+)'
$rxMix = [regex]'mix\[FR RL FL RR\]=\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)'
$rxEstGt = [regex]'est\[r p y\]=\s*([\-0-9\.]+)\s+([\-0-9\.]+)\s+[^\s]+\s+gt\[r p y\]=\s*([\-0-9\.]+)\s+([\-0-9\.]+)'

$yawRates = New-Object System.Collections.Generic.List[double]
$rot1 = New-Object System.Collections.Generic.List[int]
$rot2 = New-Object System.Collections.Generic.List[int]
$rot3 = New-Object System.Collections.Generic.List[int]
$rot4 = New-Object System.Collections.Generic.List[int]
$attErr = New-Object System.Collections.Generic.List[double]
$satCount = 0; $n=0

foreach ($l in $win) {
    $n += 1
    $m=$rxYaw.Match($l); if($m.Success){ [void]$yawRates.Add([double]$m.Groups[1].Value) }
    $m=$rxMix.Match($l); if($m.Success){
        $r1=[int]$m.Groups[1].Value; $r2=[int]$m.Groups[2].Value; $r3=[int]$m.Groups[3].Value; $r4=[int]$m.Groups[4].Value
        [void]$rot1.Add($r1); [void]$rot2.Add($r2); [void]$rot3.Add($r3); [void]$rot4.Add($r4)
        if(($r1 -le 1002) -or ($r1 -ge 1998) -or ($r2 -le 1002) -or ($r2 -ge 1998) -or ($r3 -le 1002) -or ($r3 -ge 1998) -or ($r4 -le 1002) -or ($r4 -ge 1998)){
            $satCount += 1
        }
    }
    $m=$rxEstGt.Match($l); if($m.Success){
        $er=([double]$m.Groups[1].Value - [double]$m.Groups[3].Value)
        $ep=([double]$m.Groups[2].Value - [double]$m.Groups[4].Value)
        $mag=[math]::Sqrt($er*$er + $ep*$ep)
        [void]$attErr.Add($mag)
    }
}

function Mean([double[]]$a){ if($a.Length -eq 0){ return 0 } return ($a | Measure-Object -Average).Average }
function RMS([double[]]$a){ if($a.Length -eq 0){ return 0 } $s=0.0; foreach($v in $a){ $s += $v*$v }; return [math]::Sqrt($s/$a.Length) }
function Pct([double[]]$a, [double]$p){ if($a.Length -eq 0){ return 0 } $sorted = $a | Sort-Object; $idx = [int]([math]::Floor(($sorted.Length-1) * $p)); return $sorted[$idx] }
function P2P([int[]]$arr){ if($arr.Length -eq 0){ return 0 } $min=($arr|Measure-Object -Minimum).Minimum; $max=($arr|Measure-Object -Maximum).Maximum; return ($max-$min) }

$yawAbs = @(); foreach($v in $yawRates){ $yawAbs += [math]::Abs($v) }
$yawMean = [math]::Round((Mean $yawAbs), 3)
$attRms = [math]::Round((RMS $attErr), 3)
$attP95 = [math]::Round((Pct $attErr 0.95), 3)
$p2p1 = P2P ($rot1.ToArray()); $p2p2 = P2P ($rot2.ToArray()); $p2p3 = P2P ($rot3.ToArray()); $p2p4 = P2P ($rot4.ToArray())
$p2pMax = [math]::Max([math]::Max($p2p1,$p2p2), [math]::Max($p2p3,$p2p4))
$satPct = if($n -gt 0){ [math]::Round(100.0*$satCount/$n, 2) } else { 0 }

Write-Output "file=$Path scenario=$Scenario lines=$n win=${StartSec}-${EndSec}s p2p_us=$p2pMax sat_pct=$satPct att_rms_deg=$attRms att_p95_deg=$attP95 yawR_mean_abs=$yawMean"
