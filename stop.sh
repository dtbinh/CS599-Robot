ostype=`uname`
ip='127.255.255.255'
if [[ "$ostype" == "Darwin" ]]; then
	ip='192.168.0.255'
fi
build/testCmd ${ip} 9090 -1