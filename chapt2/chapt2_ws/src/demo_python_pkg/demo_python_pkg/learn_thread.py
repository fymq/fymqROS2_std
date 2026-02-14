import threading
import requests#用于获取网址内容

class Download:
    def download(self,url,callback_word_count):
        print(f"线程:{threading.get_ident()} 开始下载:{url}")
        resqonse = requests.get(url)
        resqonse.encoding = "utf-8"
        callback_word_count(url,resqonse.text)#调用回调函数


    def start_download(self,url,callback_word_count):
        #self.download(url,callback_word_count)
        thread = threading.Thread(target=self.download,args=(url,callback_word_count))
        thread.start()

def word_count(url,result):
    print(f"{url}:{len(result)}->{result[:5]}")#普通函数，用于回调

def main():
    download = Download()
    download.start_download("http://0.0.0.0:8000/src/demo_python_pkg/resource/novel1.txt",word_count)
    download.start_download("http://0.0.0.0:8000/src/demo_python_pkg/resource/novel2.txt",word_count)
    download.start_download("http://0.0.0.0:8000/src/demo_python_pkg/resource/novel3.txt",word_count)