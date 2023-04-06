$ScriptDir = Split-Path $MyInvocation.MyCommand.Path -Parent
Set-Location $ScriptDir

# 依存関係インストール
if (-Not(Get-Command dot -ea SilentlyContinue)) {
    winget install doxygen
    $env:Path += ";C:\Program Files\doxygen\bin"

    #    $NewPath="C:\Progra~1\doxygen\bin"
    #    sudo [Environment]::SetEnvironmentVariable("Path", $env:Path + ";" + $NewPath, "Sys")
}

if (-Not(Get-Command dot -ea SilentlyContinue)) {
    winget install Graphviz.Graphviz
    $env:Path+=";C:\Program Files\Graphviz\bin"

    #    $NewPath="C:\Progra~1\Graphviz\bin"
    #    sudo [Environment]::SetEnvironmentVariable("Path", $env:Path + ";" + $NewPath, "Sys")
}

if (-Not(Test-Path $ScriptDir\Doxyfile)) {
    doxygen -g Doxyfile
}

doxygen Doxyfile