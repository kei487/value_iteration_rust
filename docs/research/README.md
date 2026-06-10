# Research notes

VI（価値反復）に関する数式研究ノート。

## ビルド

```bash
cd docs/research
pdflatex 2026-06-09-vi-ssp-acceleration.tex
pdflatex 2026-06-09-vi-ssp-acceleration.tex   # 相互参照のため2回
```

`pdflatex` 未導入なら `sudo apt-get install texlive-latex-base texlive-latex-recommended`（または `tectonic 2026-06-09-vi-ssp-acceleration.tex`）。本文は amsmath/amssymb/amsthm/booktabs/geometry/hyperref のみ依存。
