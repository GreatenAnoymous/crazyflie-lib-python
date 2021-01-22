#include <iostream>
using namespace std;


void test(int &a){
    a=1;
}

int main(){
    int a=0;
    test(a);
    cout<<a<<endl;
    return 0;
}
