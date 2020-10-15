# include <iostream>
# include <vector>

using namespace std;

int main (int argc, char** argv)
{
    for (int i = 0; i < 10000000; i++)
    {
        cout << "\rprocessing: " << (i+1) << "/1000";
    }
    cout << endl;
    for (int i = 0; i < 100; i++)
    {
        printf("\rprocessing: %.2d%/100", (i+1));
    }

    return 0;
}
// int main(int argc, char** argv){
//     vector<int> a[5];
//     for (int i = 0; i < 5; i++){
//         for (int j = i; j < 5; j++){
//             a[i].push_back(j);
//         }
//     }
//     cout << "data in" << endl;
//     for (int i = 0; i < 5; i++){
//         cout << a[i].size() << endl;
//     }
//     for (int i = 0; i < 5; i++){
//         cout << (a+i)->size() << endl;
//     }

//     return 0;
// }
// int main(int argc, char** argv)
// {
//     vector<int*> a;
//     for (int i = 0; i < 5; i++){
//         int j = i + 1;
//         int* x = &j;
//         a.push_back(x);
//     }
//     for (int i = 0; i < 5; i++){
//         cout << *a[i] << endl;
//     }
//     for (int i = 0; i < 5; i++){
//         int j = i + 10;
//         int* tmp = &j;
//         *a[i] = *tmp;
//         cout << *a[i] << endl;
//     }
// }