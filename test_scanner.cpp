#include <iostream>
#include <fstream>
#include <map>

std::map<std::string, std::string> code_product_map;
const std::string SPLIT = "|";

bool parse_data(void){
  std::ifstream f;
  f.open("database.txt", std::ifstream::in);
  if(!f){
    std::cerr << "\033[1;31mCan't open file, existing...\033[0m\n";
    return 0;
  }
  std::string line, code, product;
  while(!f.eof()){
    std::getline(f, line);
    if(line.empty()) continue;
    std::size_t found = line.find(SPLIT);
    if(found==std::string::npos){
      std::cerr << "\033[1;31mInvalid string, ignore...\033[0m\n";
      continue;
    }
    code = line.substr(0, found-1);
    product = line.substr(found+2, line.length());
    code_product_map.insert(std::pair<std::string, std::string>(code, product));
  }
  std::cout << "Read " << code_product_map.size() << " pairs: \n";
  for(auto it=code_product_map.begin();it!=code_product_map.end(); ++it){
    std::cout << "\t" << it->first << "\t: " << it->second << "\n";
  }
  return 1;
}

int main(int argc, char** argv)
{
  if(!parse_data()) return -1;
  char code[256];
  std::map<std::string, std::string>::iterator find_it;
  while(1){
    std::cin.getline(code, 256);
    std::string code_str(code);
    find_it = code_product_map.find(code_str);
    if(find_it==code_product_map.end()){
      std::cout << "Code not in database\n";
    }
    else{
      std::cout << "Product name: " << find_it->second << "\n";
    }
    std::cin.clear();
  }
  return 0;
}
