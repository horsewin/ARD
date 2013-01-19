1. Visual Studio 2010
2. Bullet Physics Library 2.78
3. TIFF-3.8.2-1
4. PNG-1.2.37
5. JPEG-6b-4 (3,4,5より先にOSGをインストールするとBUILDでエラーが出る．ここでひっかかった）
6. Boost 2.73
7. Open Scene Graph 3.0.1
8. OSGWorks 2.0.0
a)OSGの追加の部分でエラーが出て詰まった。環境変数にOSG_ROOTを追加すればよいとググったらでてきた
b)boostの部分で詰まった。最新のPCLのバージョンではなぜかBoostConfig.cmakeのファイルの中身が変で編集してもうまく動かなかったため、前PCからPCL1.4.0を回収してそれをつかうとうまく動いた。
9.OSGBullet 2.0.0
10. VRPN 7.30

