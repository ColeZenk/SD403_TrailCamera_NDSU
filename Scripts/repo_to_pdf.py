#!/usr/bin/env python3
"""
repo_to_pdf.py -- Genereates a PDF of the entire codebase for easy printing (with TOC)
Usage: python3 repo_to_pdf.py [repo_root] [output_name]
       python3 repo_to_pdf.py . myproj
"""

import os
import sys
import subprocess
import tempfile
import shutil
import argparse

# Config
EXTENSIONS = ['.c', '.h', '.v']
EXCLUDE_DIRS = {'build', 'managed_components', 'esp-idf', '.git', '__pycache__','venv', 'sim',
                'node_modules', 'components'}

AUTHOR = "Cole Zenk"
PROJECT = "SD403 Trailcam IV : Group 6"



VERILOG_DEF = r"""
\lstdefinelanguage{Verilog}{
  keywords={module, endmodule, input, output, inout, reg, wire, always, begin,
	    end, if, else, case, endcase, posedge, negedge, assign, parameter,
    	    localparam, initial, for, while, integer, task, endtask, function,
	    endfunction, default},
  sensitive=true,
  comment=[l]{//},
  morecomment=[s]{/*}{*/},
  morestring=[b]",
}
"""


# Shared lstset options
LSTSET_BASE = r"""
  numbers=left,
  numbersep=8pt,
  stepnumber=1,
  breaklines=true,
  breakatwhitespace=false,
  tabsize=4,
  showspaces=false,
  showstringspaces=false,
  extendedchars=false,
  frame=single,
  captionpos=b,
"""


def make_style(style):
    if style == 'mono':
        return VERILOG_DEF + r"\lstset{" \
               + "\n basicstyle=\\small\\ttfamily," \
               + LSTSET_BASE + "}\n"
    else:     #dark
        return r"""
\definecolor{codebg}{rgb}{0.05,0.05,0.05}
\definecolor{codegreen}{rgb}{0.3,0.8,0.3}
\definecolor{codegray}{rgb}{0.5,0.5,0.5}
\definecolor{codeblue}{rgb}{0.4,0.6,1.0}
\definecolor{codered}{rgb}{1.0,0.3,0.3}
""" + VERILOG_DEF.replace(
    " sensitive=true,",
    " keywordstyle=\\color{codeblue}\\bfseries,\n sensitive=true,\n commentstyle=\\color{codegreen}\\itshape,\n stringstyle=\\color{codered},"
) + r"\lstset{" + r"""
  backgroundcolor=\color{codebg},
  basicstyle=\footnotesize\ttfamily\color{white},
  keywordstyle=\color{codeblue}\bfseries,
  commentstyle=\color{codegreen}\itshape,
  stringstyle=\color{codered},
  numberstyle=\tiny\color{codegray},
  rulecolor=\color{codegray},
""" + LSTSET_BASE + "}\n"


def escape_latex(s):
    replacements = {
        '&': r'\&', '%': r'\%', '$': r'\$', '#': r'\#',
        '_': r'\_', '{': r'\{', '}': r'\}', '~': r'\textasciitilde{}',
        '^': r'\^{}', '\\': r'\textbackslash{}',
    }
    return ''.join(replacements.get(c, c) for c in s)


def find_files(root):
    files = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in EXCLUDE_DIRS]
        for f in sorted(filenames):
            if any(f.endswith(ext) for ext in EXTENSIONS):
                full = os.path.join(dirpath, f)
                rel  = os.path.relpath(full, root)
                files.append((rel, full))
    return sorted(files)


def lang_for_file(path):
    if path.endswith('.v'):
        return 'Verilog'
    else:
        return 'C'


def generate_latex(files, project, author, style):
    lines = []
    lines.append(r'\documentclass[10pt,a4paper]{article}')
    lines.append(r'\usepackage[utf8]{inputenc}')
    lines.append(r'\usepackage[T1]{fontenc}')
    lines.append(r'\usepackage{listings}')
    lines.append(r'\usepackage{xcolor}')
    lines.append(r'\usepackage{hyperref}')
    lines.append(r'\usepackage{geometry}')
    lines.append(r'\usepackage{inconsolata}')
    lines.append(r'\geometry{margin=0.5in}')
    lines.append(make_style(style))
    lines.append(r'\hypersetup{colorlinks=true, linkcolor=blue, urlcolor=blue}')
    lines.append(r'\title{' + escape_latex(project) + r' \\ \large Source Code Reference}')
    lines.append(r'\author{' + escape_latex(author) + '}')
    lines.append(r'\date{\today}')
    lines.append(r'\begin{document}')
    lines.append(r'\maketitle')
    lines.append(r'\tableofcontents')
    lines.append(r'\newpage')

    for rel, full in files:
        lang = lang_for_file(rel)
        section_title = escape_latex(rel)
        lines.append(r'\section*{' + section_title + '}')
        lines.append(r'\addcontentsline{toc}{section}{' + section_title + '}')
        lines.append(r'\lstinputlisting[language=' + lang + ']{' + full + '}')
        lines.append(r'\newpage')

    lines.append(r'\end{document}')
    return '\n'.join(lines)



def main():
    parser = argparse.ArgumentParser(description='Generate a PDF of your codebase.')
    parser.add_argument('root', nargs='?', default='.', help='Repo root (def: .)')
    parser.add_argument('outname', nargs='?', default='codebase', help='Output filename without .pdf')
    parser.add_argument('--style', choices=['mono', 'dark'], default='mono',
                        help='mono: Bell Labs style (default), dark: syntax highlighted dark theme')
    args = parser.parse_args()
    root = os.path.abspath(args.root)

    print(f"Scanning {root}... (style: {args.style})")
    files = find_files(root)
    print(f"Found {len(files)} files:")
    for rel, _ in files:
        print(f"  {rel}")

    latex = generate_latex(files, PROJECT, AUTHOR, args.style)

    with tempfile.TemporaryDirectory() as tmpdir:
        tex_path = os.path.join(tmpdir, 'output.tex')
        with open(tex_path, 'w', encoding='utf-8', errors='replace') as f:
            f.write(latex)

        print("\nCompiling PDF...")
        for _ in range(2):                # Run twice for TOC
            result = subprocess.run(
                ['pdflatex',
                 '-interaction=nonstopmode',
                 '-output-directory',
                 tmpdir,
                 tex_path],
                capture_output=True, text=True
            )

        pdf_src = os.path.join(tmpdir, 'output.pdf')
        pdf_dst = os.path.join(os.getcwd(), args.outname + '.pdf')

        if os.path.exists(pdf_src):
            shutil.copy(pdf_src, pdf_dst)
            print(f"\nDone: {pdf_dst}")
        else:
            print("pdflatex failed. Log:")
            print(result.stdout[-2000:])


if __name__ == '__main__':
    main()
