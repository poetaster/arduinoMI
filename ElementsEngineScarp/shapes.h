#include <iostream>


/*
int main() {
    const char* const list[] = {"zip", "zam", "bam"};
    const size_t len = sizeof(list) / sizeof(list[0]);

    for (size_t i = 0; i < len; ++i)
        std::cout << list[i] << "\n";

    const std::vector<std::string> v(list, list + len);
    std::copy(v.begin(), v.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
}*/
  
  
const char* const oscnames[] = {
"BOW-1",
"BLOW1",
"BLOWC",
"MTAL1",
"STRK1",
"MALTS",
"MATSP",
};
 const size_t nameslength = sizeof(oscnames) / sizeof(oscnames[0]);
