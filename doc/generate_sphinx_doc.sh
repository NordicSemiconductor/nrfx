rm -rf html xml html_sphinx
doxygen nrfx.doxyfile
sphinx-build -b html sphinx html_sphinx
