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
}

# フォルダの有無で初期化実行
if (-Not(Test-Path $ScriptDir\sphinx)) {
    sphinx-quickstart sphinx
}

# ドキュメント生成
sphinx-apidoc -e -f -o $ScriptDir\sphinx .
sphinx-build -a $ScriptDir\sphinx $ScriptDir\docs
