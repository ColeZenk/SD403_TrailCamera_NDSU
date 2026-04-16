#!/bin/sh
# Generate PDFs from all SDD markdown files
# Usage: ./gen_sdds.sh [--long]
#   --long  single continuous page per document (no page breaks)

OUTDIR="./Documentation/SDDs/pdfs"
LONG=0

if [ "$1" = "--long" ]; then
  LONG=1
  OUTDIR="./Documentation/SDDs/pdfs/long"
fi

mkdir -p "$OUTDIR"

fail=0
for f in $(find . -type f -name "SDD*.md" | sort); do
  base=$(basename "${f%.md}")

  printf "%-50s -> %s.pdf\n" "$f" "$base"
  if [ "$LONG" = "1" ]; then
    pandoc "$f" \
      -V geometry:paperwidth=8.5in \
      -V geometry:paperheight=200in \
      -V geometry:margin=0.75in \
      -V pagestyle=empty \
      -F mermaid-filter \
      -o "${OUTDIR}/${base}.pdf" 2>&1 \
    && pdfcrop "${OUTDIR}/${base}.pdf" "${OUTDIR}/${base}.pdf" >/dev/null 2>&1 \
    || fail=$((fail + 1))
  else
    pandoc "$f" -V geometry:margin=0.75in -F mermaid-filter -o "${OUTDIR}/${base}.pdf" 2>&1 || fail=$((fail + 1))
  fi
done

total=$(find . -type f -name "SDD*.md" | wc -l)
echo ""
echo "Done: $((total - fail))/${total} succeeded"
