<A indication for installing the AR Diorama server>
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

<A limiation of VRPN connection>
Ref:http://www.cs.unc.edu/Research/vrpn/Connection_h.html
-->
// Buffer lengths for TCP and UDP.
// TCP is an arbitrary number that can be changed by the user
// using vrpn_Connection::set_tcp_outbuf_size().
// UDP is set based on Ethernet maximum transmission size;  trying
// to send a message via UDP which is longer than the MTU of any
// intervening physical network may cause untraceable failures,
// so for now we do not expose any way to change the UDP output
// buffer size.  (MTU = 1500 bytes, - 28 bytes of IP+UDP header)
const	int vrpn_CONNECTION_TCP_BUFLEN = 64000;
const	int vrpn_CONNECTION_UDP_BUFLEN = 1472;
/// Number of endpoints that a server connection can have.  Arbitrary limit.
<--

2013.1.30までUDP通信しか使っていなかった。そのためバッファ制限が1472byteに制限されており十分なデータ転送が
できず、a set of hand spheresなどを完全にクライアント側で構築できなかった。
ドキュメントを読んでみると以下のような記述を見つけた。
Ref: http://www.cs.unc.edu/Research/vrpn/troubleshooting.html

No "unreliable" messages seem to get through, can't send UDP
Step 1: upgrade to version 7.15 or higher; there was a bug in earlier versions that caused this to happen on some computers with more than one network connection.
Step 2: We had trouble with VRPN when going through firewalls at high schools. To fix it, we implemented the TCP-only connection, you can connect to device@tcp://machine:port rather than device@machine:port, and the client will then make a single outgoing TCP connection to the server and no unreliable (UDP) channel will be established between them. This should also work to avoid firewalls on the client computer.

バッファを拡張し、クライアント側の通信形態をTCPに変更すると、サーバ側で設定したバッファ内容を正しく受信できた。
これによって、a set of hand spheresを完全に構成することに成功した。