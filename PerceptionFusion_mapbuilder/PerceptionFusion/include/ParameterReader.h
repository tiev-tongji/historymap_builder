//
// Created by xlz on 17-10-20.
//

#ifndef TEST_CARTOGRAPHER_PARAMETERREADER_H
#define TEST_CARTOGRAPHER_PARAMETERREADER_H
#include <fstream>
#include <vector>
#include <map>

using namespace std;
class ParameterReader
{
public:
    ParameterReader( string filename )
    {
        if (filename == "")
        {
            filename = "../parameters.txt";
        }
        ifstream fin( filename.c_str() );
        if (!fin)///保证存在这个文件
        {
            std::cerr<<"parameter file does not exist."<<endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                /// 以‘＃’开头的是注释
                continue;
            }

            int index = str.find("=");///寻找有'='的行，返回‘=’第一次出现的位置
            if (index == -1)
                continue;
            ///‘=’左边为参数变量，右边为对应的值
            string key = str.substr( 0, index );
            string value = str.substr( index+1, str.length() );
            data[key] = value;

            if ( !fin.good() )///保证输入的是有效字符串
                break;
        }
    }
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);

        if (iter == data.end())
        {
            cerr<<data[key]<<endl;
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }
private:
    map<string, string> data;
};


#endif //TEST_CARTOGRAPHER_PARAMETERREADER_H
