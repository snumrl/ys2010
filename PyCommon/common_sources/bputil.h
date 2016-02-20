#pragma once

//#include <boost/python.hpp>
//#include <python.h>
//using namespace boost::python;

#define XD(v)	((extract<double>)(v))
#define XC(v)	((extract<char*>)(v))
#define XS(v)	((extract<string>)(v))
#define XF(v)	((extract<float>)(v))
#define XI(v)	((extract<int>)(v))
#define XB(v)	((extract<bool>)(v))



//inline void printSO3(const object& SO3)
//{
//	cout << "[ ";\
//	for(int i=0; i<len(SO3); ++i)\
//	{\
//		if(i>0)\
//			cout << "  ";\
//		for(int j=0; j<len(SO3[i]); ++j)\
//			cout << XD(SO3[i][j]) << ' ';\
//		if(i==len(SO3)-1)\
//			cout << "]";\
//		cout << endl;\
//	}\
//}

//inline void Tokenize(const string& str,
//                      vector<string>& tokens,
//                      const string& delimiters = " ")
//{
//    // �� ù ���ڰ� �������� ��� ����
//    string::size_type lastPos = str.find_first_not_of(delimiters, 0);
//    // �����ڰ� �ƴ� ù ���ڸ� ã�´�
//    string::size_type pos     = str.find_first_of(delimiters, lastPos);
//
//    while (string::npos != pos || string::npos != lastPos)
//    {
//        // token�� ã������ vector�� �߰��Ѵ�
//        tokens.push_back(str.substr(lastPos, pos - lastPos));
//        // �����ڸ� �پ�Ѵ´�.  "not_of"�� �����϶�
//        lastPos = str.find_first_not_of(delimiters, pos);
//        // ���� �����ڰ� �ƴ� ���ڸ� ã�´�
//        pos = str.find_first_of(delimiters, lastPos);
//    }
//}
//