# rs_docs
Source of the RoboSherlock project webpage (www.robosherlock.org)

Install dependencies:
```
# Assuming you have a python 3 install (for example in a venv/anaconda/etc.)
pip3 install -U Sphinx
pip3 install sphinxcontrib-bibtex sphinxcontrib-googleanalytics
```
or on Linux with a systemwide python install (legacy):
```
sudo apt-get install python-sphinx python-pip
sudo pip install sphinxcontrib-bibtex sphinxcontrib-googleanalytics
# If you get an error that Sphinx>=2.0 can't be found in the last step, install the older version of sphinxcontrib-bibtex by executing:
#   sudo pip install sphinxcontrib-bibtex==0.4.2
```
## Build the website
Just execute 'make html' in the repo after the installation described above.
