# nrfx documentation

## Doxygen

You can generate `doxygen` based documentation by running

```shell
doxygen nrfx.doxyfile
```

You may want to use the provided scripts `generate_html_doc.sh` or
`generate_html_doc.bat`. The result can be viewed by opening
`html/index.html`.

## Sphinx

All the necessary files to compile the Sphinx based documentation for `nrfx`
are located under `sphinx` folder. As of today the content should match with
the one produced using `doxygen` only.

### Requirements

You will need to have Python 3 installed as well as some dependencies, which can
be installed by running:

```shell
pip install -r requirements.txt
```

### Build

You may want to use the provided scripts `generate_sphinx_doc.sh` or
`generate_sphinx_doc.bat`. The result can be viewed by opening
`html_sphinx/index.html`.

If you want to do it manually you can run the following commands from `doc`
directory:

```shell
# compile doxygen documentation (required to generate XML metadata)
doxygen nrfx.doxyfile
# compile Sphinx documentation
sphinx-build -b html sphinx html_sphinx
```