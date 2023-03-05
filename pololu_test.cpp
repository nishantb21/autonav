#include <fstream> //for ofstream
#include <string>
#include <unistd.h> //for sleep()

using namespace std;

class pololuComms{
    ofstream* pololu_stream;
};

pololuComms::pololuComms(string pololu_port){
    *pololu_stream.open(pololu_port);
}

int pololuComms::sendPololuCmd(int channel, int pulse_width){
    int target = pulse_width*4;
    char bytes[4];
    bytes[0] = 0x84;
    bytes[1] = channel;
    bytes[2] = target & 0x7f;
    bytes[3] = (target>>7) & 0x7f;
    
    pololu_stream->write(bytes)
    return 0;
}

int pololuComms::wave(int channel){
    sendPololuCmd(channel,1000);
    sleep(2.5);
    sendPololuCmd(channel,2000);
    sleep(2.5);
    sendPololuCmd(channel,1500);
    sleep(2.5);
}

int main(){
    pololu = pololuComms('dev/tty/ACM0');
    pololu.wave(0);    
    return 0;
}