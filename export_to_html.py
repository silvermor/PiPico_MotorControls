import os
import time
import getpass
import argparse

from IPython.display import display
from IPython.display import Javascript
from IPython.display import Markdown as md

time_stamp = ""

def do_do_export(do_timestamp=False): 
    if do_timestamp:
        cmd = 'IPython.notebook.save_notebook();' \
              'IPython.notebook.kernel.execute(' \
              '"do_export_ts(" + "\'" + ' \
              'IPython.notebook.notebook_name + "\')");'
    else:
        cmd = 'IPython.notebook.save_notebook();' \
              'IPython.notebook.kernel.execute(' \
              '"do_export(" + "\'" + ' \
              'IPython.notebook.notebook_name + "\')");'
    display(Javascript(cmd))

def do_export_ts(notebookName):
    do_export(notebookName, True)
    
def do_export(notebookName, do_timestamp=False):
    suffix = ".html"
    if do_timestamp:
        ts = time.strftime("%Y%m%d_%H%M")
        suffix = "_"+ts+".html"
        global time_stamp
        time_stamp = ts
    notebookHtml = notebookName.replace(".ipynb", suffix)
    print("nb=%s html=%s"%(notebookName, notebookHtml))
    cmd = "jupyter nbconvert --to=html --output=%s %s"%(
        notebookHtml, notebookName)
    print(cmd)
    time.sleep(2)
    os.system(cmd)
    user = getpass.getuser()
    cmd = "cp -fp %s /mnt/uzed/www/%s/"%(notebookHtml, user)
    print(cmd)
    os.system(cmd)
    print(time.ctime())
    url = "http://positron.hep.upenn.edu/uzed/%s/%s"%(
        user, notebookHtml)
    s = """
    * exported notebooks: http://positron.hep.upenn.edu/uzed/
    * HTML rendering of notebook should appear at <{}>
    """
    s = s.format(url)
    os.unlink(notebookHtml)

if __name__=="__main__":
    parser = argparse.ArgumentParser(
        description="export active notebook to html")
    parser.add_argument("--timestamp", action="store_true",
                        help="append timestamp to html filename")
    args = parser.parse_args()
    do_do_export(do_timestamp=args.timestamp)
    ts = time.strftime("%Y%m%d_%H%M")
    print("approximate time stamp:", ts)
