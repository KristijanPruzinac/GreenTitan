$ErrorActionPreference = 'Stop'
$workspaceRoot = 'C:\Users\kikom\Documents\GitHub\GreenTitan'
$generatedRoot = 'P:\GreenTitan-CAD-work'
$referenceRoot = Join-Path $workspaceRoot 'outputs\chassis_reference'
$expectedReferences = @{
  'original_v3_context.step' = 'F19D623EA47CEC435B83EEC787EA7425343F398B055C4625C52260883089E7A7'
  'source_fingerprint.json' = 'DEB2DD4C1B31AE2598D1DE907A98C35D6F3DE61B262D9B11E39DDD28949D8F44'
  'BladeSpace_local.step' = '60C50EEEE3B1EE834921F1C92AEDF578727593EB15A7A9FDCD0157A301C1F1D1'
  'BladeSpace_transform.json' = '22C590D01F28BF2A7DB3B7A36C8B27450EE40CA14ED36F494EC4199BD68E628E'
}
foreach ($name in $expectedReferences.Keys) {
  if ((Get-FileHash -LiteralPath (Join-Path $referenceRoot $name) -Algorithm SHA256).Hash -ne $expectedReferences[$name]) {
    throw ('Source reference verification failed: ' + $name)
  }
}
$plans = @()
foreach ($version in 'roof_v4','roof_v5','roof_v6') {
  $target = [IO.Path]::GetFullPath((Join-Path $generatedRoot $version))
  $code = [IO.Path]::GetFullPath((Join-Path $workspaceRoot ('cad\' + $version)))
  $link = [IO.Path]::GetFullPath((Join-Path $workspaceRoot ('outputs\' + $version)))
  if ($target -ne ($generatedRoot + '\' + $version) -or $code -ne ($workspaceRoot + '\cad\' + $version)) { throw 'Unexpected absolute target' }
  $linkItem = Get-Item -LiteralPath $link -Force
  if ($linkItem.LinkType -ne 'Junction' -or @($linkItem.Target)[0] -ne $target) { throw 'Unexpected output junction target' }
  foreach ($tree in @($target,$code)) {
    $treeItem = Get-Item -LiteralPath $tree -Force
    if ($treeItem.Attributes -band [IO.FileAttributes]::ReparsePoint) { throw ('Removal tree itself is a link: ' + $tree) }
    $nestedLinks = @(Get-ChildItem -LiteralPath $tree -Recurse -Force | Where-Object { $_.Attributes -band [IO.FileAttributes]::ReparsePoint })
    if ($nestedLinks.Count -gt 0) { throw ('Nested links require inspection: ' + $tree) }
  }
  $plans += [pscustomobject]@{version=$version; target=$target; code=$code; link=$link}
}
$results = @()
foreach ($plan in $plans) {
  $bytes = (Get-ChildItem -LiteralPath $plan.target -Recurse -File -Force | Measure-Object -Property Length -Sum).Sum
  Remove-Item -LiteralPath $plan.target -Recurse -Force
  Remove-Item -LiteralPath $plan.code -Recurse -Force
  # DirectoryInfo.Delete(false) removes only the now-dangling junction itself.
  [IO.Directory]::Delete($plan.link, $false)
  $results += [pscustomobject]@{version=$plan.version; output_deleted=$plan.target; code_deleted=$plan.code; junction_deleted=$plan.link; output_bytes=$bytes}
}
$record = [pscustomobject]@{authorization='User requested deleting rejected concepts and starting from scratch'; source_reference_hashes_verified=$true; removed=$results; original_fusion_source_preserved=$true}
$record | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $referenceRoot 'rejected_roofs_removed.json') -Encoding UTF8
$record | ConvertTo-Json -Depth 5
