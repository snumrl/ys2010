// SVMWrap.cpp: implementation of the CSVMWrap class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "SVMWrap.h"
#include "../utility/util.h"

void parse_command_line(int argc, char **argv, svm_parameter& param);	/// svmtrain.c�� ���� ���� �״�� �����.
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CSVMWrap::CSVMWrap(CTArray<matrixn>& data, bool bNormalize)
{
	m_nOneDataNumRow=data[0].rows();	// oneData�κ��� �ٽ� matrix�� �����ϱ� ���ؼ� �ʿ�.
	
	// Make one very big matrix
	matrixn AllData;
	vectorn oneData;
	oneData.setSize(data[0].rows()*data[0].cols());
	AllData.setSize(data.Size(), oneData.size());
	
	for(int i=0; i<data.Size(); i++)
	{
		data[i].toVector(oneData);
		AllData.setRow(i, oneData);
	}
	
	Initialize(AllData,bNormalize);
}

CSVMWrap::CSVMWrap(matrixn& data, bool bNormalize, int oneDataNumRow)
{
	m_nOneDataNumRow=oneDataNumRow;
	Initialize(data,bNormalize);
}

void min_max_value(const matrixn& data, vectorn& minX, vectorn& maxX)
{
	int row, col;
	minX.setSize(data.cols());
	maxX.setSize(data.cols());

	for(col=0; col<data.cols(); col++)
	{
		minX[col]=INT_MAX;
		maxX[col]=INT_MIN;
	}

	for(row=0; row<data.rows(); row++)
	{
		for(col=0; col<data.cols(); col++)
		{
			if(data[row][col]<minX[col])
				minX[col]=data[row][col];
			if(data[row][col]>maxX[col])
				maxX[col]=data[row][col];
		}
	}
}

void CSVMWrap::Initialize(matrixn& data, bool bNormalize)
{
	vectorn minX;
	vectorn inv_range;
	if(bNormalize)
	{
		//ScaleData(data);	// 0, 1 unit hyper cube �� scale
		ASSERT(data.isValid());
		int row, col;
		
		vectorn maxX;


		min_max_value(data, minX, maxX);
		
		inv_range=maxX;
		inv_range-=minX;
		for(col=0; col<data.cols(); col++)
		{
			if(inv_range[col]<=0.0000001)
			{
				minX[col]-=0.5;
				inv_range[col]=1.0;
			}
			else
				inv_range[col]=1.0/inv_range[col];
		}

	}

	m_sAllData.l = data.rows();
	int elements = (data.cols()+1)*data.rows();

	m_sAllData.y = new double[m_sAllData.l];
	m_sAllData.x = new svm_node*[m_sAllData.l];
	m_xspace = new svm_node[elements];

	int max_index = data.cols();	// sparse�� ����Ÿ�� ��� ����Ÿ�� 0�̾ƴ� ���� �ִ� ������ ������, �� Ŭ���������� sparse����Ÿ�� ������� �����Ƿ� �׳� �÷����̴�.
	int i,j, count;
	count=0;
	for(i=0;i<m_sAllData.l;i++)
	{
		m_sAllData.x[i] = &m_xspace [count];
		m_sAllData.y[i] = -1.0;	// not defined class.

		for(j=0; j< data.cols(); j++)
		{
			m_xspace [count].index=j+1;	// 1���� �����ϴ� index�� �ٲپ� �־�� �Ѵ�.

			if(bNormalize)
				m_xspace [count].value=(data[i][j]-minX[j])*inv_range[j];
			else
				m_xspace [count].value=data[i][j];
			count++;
		}
		m_xspace [count++].index = -1;
	}
	ASSERT(count==elements);
	fGamma = 1.0/max_index;
}

void CSVMWrap::GetMatrix(const svm_node* pNode, int nMatrixRow, matrixn& mat)
{
	int countColumn;
	for(countColumn=0; pNode[countColumn].index!=-1; countColumn++);

	ASSERT(countColumn%nMatrixRow==0);

	int nMatrixColumn=countColumn/nMatrixRow;

	mat.setSize(nMatrixRow, nMatrixColumn);

	for(int i=0; i<nMatrixRow; i++)
	{
		for(int j=0; j<nMatrixColumn; j++)
		{
			mat[i][j]=pNode[i*nMatrixColumn+j].value;
		}
	}
}

void CSVMWrap::GetVector(const svm_node* pNode, vectorn& vec)
{
	int countColumn;
	for(countColumn=0; pNode[countColumn].index!=-1; countColumn++);

	vec.setSize(countColumn);

	for(int i=0; i<countColumn; i++)
	{
		vec[i]=pNode[i].value;
	}
}

CSVMWrap::~CSVMWrap()
{
	delete[] m_sAllData.y;
	delete[] m_xspace;
	delete[] m_sAllData.x;
}

void CSVMWrap::TrainAndPredict(const char* option)
{
	int argc;
	char** argv;
	svm_parameter param;
	svm_problem prob;
	svm_model *model;
	const char *error_msg;

	ParseCommandLineString(option,argc, argv);
	parse_command_line(argc, argv, param);
	FreeCommandLineString(argc, argv);
	
	if(param.gamma==0) param.gamma=fGamma;
	if(m_nOneDataNumRow) param.degree=m_nOneDataNumRow;

	prob.l=m_setTrainingData.size();
	prob.x=new svm_node*[m_setTrainingData.size()];
	prob.y=new double[m_setTrainingData.size()];

	for(int i=0; i<m_setTrainingData.size(); i++)
	{
		prob.x[i]=m_sAllData.x[m_setTrainingData[i].hData];
		prob.y[i]=m_setTrainingData[i].hClass;
	}

	error_msg = svm_check_parameter(&prob,&param);

	if(error_msg)
	{
		TRACE("Error: %s\n",error_msg);
		ASSERT(0);
		return;
	}

	model = svm_train(&prob,&param);

	for(int i=0; i<m_sAllData.l ; i++)
	{
		m_sAllData.y[i]=svm_predict(model, m_sAllData.x[i]);
	}

//	svm_save_model(model_file_name,model);
	svm_destroy_model(model);

	delete[] prob.y;
	delete[] prob.x;
}


void CSVMWrap::ScaleData(matrixn& data)
{
	ASSERT(data.isValid());
	int row, col;
	vectorn minX;
	vectorn maxX;

	min_max_value(data, minX, maxX);

	vectorn inv_range;
	inv_range=maxX;
	inv_range-=minX;
	for(col=0; col<data.cols(); col++)
		inv_range[col]=1.0/inv_range[col];
	
	for(row=0; row<data.rows(); row++)
	{
		for(col=0; col<data.cols(); col++)
		{
			data[row][col]-=minX[col];
			data[row][col]*=inv_range[col];
		}
	}
}


void parse_command_line(int argc, char **argv, svm_parameter& param)
{
	int i;

	// default values
	param.svm_type = C_SVC;
	param.kernel_type = RBF;
	param.degree = 3;
	param.gamma = 0;	// 1/k
	param.coef0 = 0;
	param.nu = 0.5;
	param.cache_size = 40;
	param.C = 1;
	param.eps = 1e-3;
	param.p = 0.1;
	param.shrinking = 1;
	param.nr_weight = 0;
	param.weight_label = NULL;
	param.weight = NULL;

	// parse options
	for(i=0;i<argc;i++)
	{
		if(argv[i][0] != '-') break;
		++i;
		switch(argv[i-1][1])
		{
			case 's':
				param.svm_type = atoi(argv[i]);
				break;
			case 't':
				param.kernel_type = atoi(argv[i]);
				break;
			case 'd':
				param.degree = atof(argv[i]);
				break;
			case 'g':
				param.gamma = atof(argv[i]);
				break;
			case 'r':
				param.coef0 = atof(argv[i]);
				break;
			case 'n':
				param.nu = atof(argv[i]);
				break;
			case 'm':
				param.cache_size = atof(argv[i]);
				break;
			case 'c':
				param.C = atof(argv[i]);
				break;
			case 'e':
				param.eps = atof(argv[i]);
				break;
			case 'p':
				param.p = atof(argv[i]);
				break;
			case 'h':
				param.shrinking = atoi(argv[i]);
				break;
			case 'v':
				
				ASSERT(0);
				/*cross_validation = 1;
				nr_fold = atoi(argv[i]);
				if(nr_fold < 2)
				{
					fprintf(stderr,"n-fold cross validation: n must >= 2\n");
					exit_with_help();
				}*/
				break;
			case 'w':
				++param.nr_weight;
				param.weight_label = (int *)realloc(param.weight_label,sizeof(int)*param.nr_weight);
				param.weight = (double *)realloc(param.weight,sizeof(double)*param.nr_weight);
				param.weight_label[param.nr_weight-1] = atoi(&argv[i-1][2]);
				param.weight[param.nr_weight-1] = atof(argv[i]);
				break;
			default:
				fprintf(stderr,"unknown option\n");
				ASSERT(0);
		}
	}

}


