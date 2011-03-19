#include <Galil.h>
#include <iostream>
#include <string>
#include <vector>
#include <sys/time.h>
#include "accelera_record.h"
#include "util.h"

#define IP_ADDR "192.168.1.105 UDP"

using namespace std;

Galil* make_galil() {
    return new Galil(IP_ADDR);
}

int main (int argc, char** argv) {
    vector<string> addresses = Galil::addresses();
    cout << "The following addresses are available:";
    for(int i=0;i<addresses.size();i++) {
        cout << "\t" << addresses.at(i);
    }
    cout << endl;
    try {
        cout << "Attempting to connect to DMC-4080 @ " << IP_ADDR << endl;
        Galil* g = make_galil();
        cout << g->connection() << endl;

        cout << "Attempting to send message to DMC-4080" << endl;
        g->command("TM 187.5");
        g->command("KP*=0");
        g->command("KI*=0");
        g->command("KD*=0");
        g->command("DR 5.33");
        string s = g->command("OF*=0");
        cout << "Message success: " << s << endl;
        Accelera_record* rec = new Accelera_record(g);

        timeval now, then, a, b;
        gettimeofday(&now, NULL);
        gettimeofday(&then, NULL);
        gettimeofday(&b, NULL);
        int volt = 4;
        for(int i=0;i<3000;i++) {

            then = now;
            gettimeofday(&now, NULL);
            double diff = (((double) now.tv_sec) * 1000000 + now.tv_usec) - (((double) then.tv_sec) * 1000000 + then.tv_usec);
            cout << "Time elapsed: " << diff << endl;

            gettimeofday(&a, NULL);
            double timer = (((double) a.tv_sec) * 1000000 + a.tv_usec) - (((double) b.tv_sec) * 1000000 + b.tv_usec);
            if (timer > 10000) {
                volt *= -1;
                int old_timeout = g->timeout_ms;
                g->timeout_ms   = 0;
                try {
                    g->command(build_cmd("OF", 'F', 'c', volt, 'i'));
                } catch (...) { }
                g->timeout_ms = old_timeout;
                cout << "Voltage away" << endl;
                gettimeofday(&b, NULL);
            }

            vector<char> record = rec->get_record();
            for(char j='A';j<='H';j++) {
                char command[10];
                sprintf(command, "_TP%c", j);
                double d = g->sourceValue(record, command);
                cout << command << "-(" << d << ")" << endl;
            }
        }
        g->command("OF ,,,,,0");
        delete g;

    } catch (string e) {
        cout << "Bad news: " << e << endl;
    }
    return 0;
}
