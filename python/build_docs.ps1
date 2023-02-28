# conda 有効化
Invoke-Expression -Command $env:UserProfile\miniforge3\shell\condabin\conda-hook.ps1
conda activate py38pip

$ScriptDir = Split-Path $MyInvocation.MyCommand.Path -Parent
Set-Location $ScriptDir

# 依存関係インストール
#pip install sphinx sphinx_rtd_theme sphinx-mdinclude
if (-Not(Get-Command dot -ea SilentlyContinue)) {
    winget install Graphviz.Graphviz
    $env:Path+=";C:\Program Files\Graphviz\bin"

#    $NewPath="C:\Progra~1\Graphviz\bin"
#    sudo [Environment]::SetEnvironmentVariable("Path", $env:Path + ";" + $NewPath, "Sys")
}

# フォルダの有無で初期化実行
if (-Not(Test-Path $ScriptDir\sphinx)) {
    $ProjectName = Convert-Path $ScriptDir\.. | Split-Path -Leaf
    sphinx-quickstart -q -p $ProjectName -a applejxd -v 0.1 -l ja sphinx
}

# ドキュメント生成
sphinx-apidoc -e -f -o $ScriptDir\sphinx .
sphinx-build -a $ScriptDir\sphinx $ScriptDir\docs

#if (-Not(Get-Command latexmk -ea SilentlyContinue)) {
#    sudo choco install texlive -y
#}
Set-Location $ScriptDir\sphinx
make latexpdfja
Set-Location $ScriptDir