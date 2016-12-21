#include "counter.h"

/*#pragma hdrstop
#pragma argsused

#include <tchar.h>
#include <stdio.h>
#include <iostream.h>
#include <string.h>



typedef enum
	{
	none = 0,
	plas,
	minus,
	multiply,
	devide,
	value,
	equal,
	notEqual,
	less,
	more,
	and,
	or,
	powO,
	brackets
	} tdNodeType;

typedef struct tdnNnode
	{
	bool enable;
	tdNodeType type;
	float value;
	tdnNnode *pLeft;
	tdnNnode *pRight;
	string name;
	} tdNode;

typedef struct
	{
	string name;
	tdNode *pNode;
	float counter;
	} tdCounter;


float checkLeafs(tdNode *_pLeaf)
	{
	if( _pLeaf == NULL )
		return 0.0;

	switch( _pLeaf->type )
		{
		case tdNodeType::equal:
			 return checkLeafs( _pLeaf->pLeft ) == checkLeafs( _pLeaf->pRight );
			 break;

		case tdNodeType::notEqual:
			 return checkLeafs( _pLeaf->pLeft ) != checkLeafs( _pLeaf->pRight );
			 break;

		case tdNodeType::less:
			 return checkLeafs( _pLeaf->pLeft ) < checkLeafs( _pLeaf->pRight );
			 break;

		case tdNodeType::more:
			 return checkLeafs( _pLeaf->pLeft ) > checkLeafs( _pLeaf->pRight );
			 break;

		case tdNodeType::plas:
			 return checkLeafs( _pLeaf->pLeft ) + checkLeafs( _pLeaf->pRight );
			 break;

		case tdNodeType::minus:
			 return checkLeafs( _pLeaf->pLeft ) - checkLeafs( _pLeaf->pRight );
			 break;

		case tdNodeType::multiply:
			 return checkLeafs( _pLeaf->pLeft ) * checkLeafs( _pLeaf->pRight );
			 break;

		case tdNodeType::devide:
			 return checkLeafs( _pLeaf->pLeft ) / checkLeafs( _pLeaf->pRight );
			 break;

		case tdNodeType::and:
			 return checkLeafs( _pLeaf->pLeft ) && checkLeafs( _pLeaf->pRight );
			 break;

		case tdNodeType::or:
			 return checkLeafs( _pLeaf->pLeft ) || checkLeafs( _pLeaf->pRight );
			 break;

		case tdNodeType::value:
		default:
			 return _pLeaf->value;
			 break;

		}
	return 0.0;
	}

float checkLeafs2(tdNode *_pLeaf)
	{
	if( _pLeaf == NULL )
		return 0.0;

	checkLeafs( _pLeaf->pLeft );
	checkLeafs( _pLeaf->pRight );

	switch( _pLeaf->type )
		{
		case tdNodeType::equal:
			 _pLeaf->value = _pLeaf->pLeft->value == _pLeaf->pRight->value;
			 break;

		case tdNodeType::notEqual:
			 _pLeaf->value = _pLeaf->pLeft->value != _pLeaf->pRight->value;
			 break;

		case tdNodeType::less:
			 _pLeaf->value = _pLeaf->pLeft->value < _pLeaf->pRight->value;
			 break;

		case tdNodeType::more:
			 _pLeaf->value = _pLeaf->pLeft->value > _pLeaf->pRight->value;
			 break;

		case tdNodeType::plas:
			 _pLeaf->value = _pLeaf->pLeft->value + _pLeaf->pRight->value;
			 break;

		case tdNodeType::minus:
			 _pLeaf->value = _pLeaf->pLeft->value - _pLeaf->pRight->value;
			 break;

		case tdNodeType::multiply:
			 _pLeaf->value = _pLeaf->pLeft->value * _pLeaf->pRight->value;
			 break;

		case tdNodeType::devide:
			 _pLeaf->value = _pLeaf->pLeft->value / _pLeaf->pRight->value;
			 break;

		case tdNodeType::and:
			 _pLeaf->value = _pLeaf->pLeft->value && _pLeaf->pRight->value;
			 break;

		case tdNodeType::or:
			 _pLeaf->value = _pLeaf->pLeft->value || _pLeaf->pRight->value;
			 break;

		case tdNodeType::value:
		default:
			 return _pLeaf->value;
			 break;

		}
	return 0.0;
	}






void parser(const string &_equation, tdNode *_pLeaf)
	{
	int pos, pos1, pos2;
	static int i = 0;
	//cout<<i++<<endl;
	string oper;
	string equation;
	string operatorsTxt[] =
		{
		"+",
		"-",
		"/",
		"*",
		"^",
		"pow",
		"or",
		"||",
		"and",
		"&&",
		"equal",
		"notequal",
		"more",
		"less",
		"("
		")"
		};

	tdNodeType operatorsTypes[] =
		{
		tdNodeType::plas,
		tdNodeType::minus,
		tdNodeType::devide,
		tdNodeType::multiply,
		tdNodeType::powO,
		tdNodeType::or,
		tdNodeType::or,
		tdNodeType::and,
		tdNodeType::and,
		tdNodeType::equal,
		tdNodeType::notEqual,
		tdNodeType::less,
		tdNodeType::more,
		tdNodeType::brackets,
		tdNodeType::brackets
		};

	if( _equation.length() == 0 )
		{
		_pLeaf = NULL;
		cout<<"NULL"<<endl;
		return;
		}
	cout<<"oh shit"<<endl;
	int operatorsAmount = sizeof(operatorsTxt) / sizeof(operatorsTxt[0]);


	int len = _equation.length();
	for( int i = 0; i < len; i++ )
		{
		equation += tolower( _equation.at( i ) ) ;
		}
	//cout<<equation.c_str()<<endl;
	if( _pLeaf == NULL )
		{
		_pLeaf = (tdNode*)malloc(sizeof(tdNode));
		_pLeaf->type = tdNodeType::none;
		_pLeaf->value = 0.0;
		cout<<"New Leaf"<<endl;
		}

	//cout<<operatorsAmount<<endl;
	int k = 0;
	while( k < operatorsAmount && equation.empty() == false )
		{
		oper = operatorsTxt[ k ];
		pos = equation.find_first_of( oper );
		if( pos != string::npos )
			{
			cout<<"oh"<<endl;
			_pLeaf->type = operatorsTypes[ k ];
			parser(equation.substr(0, pos), _pLeaf->pLeft);
			parser(equation.substr(pos, equation.length()),  _pLeaf->pRight);
			equation.erase(0, pos);
			equation.erase(pos, equation.length());
			}
		k++;
		}
	if( equation.empty() == false )
		{
		pos = equation.find_first_of("0123456789");
		if( pos != string::npos )
			{
			_pLeaf->type = tdNodeType::value;
			_pLeaf->value = atoi( _equation.c_str() );
			}
        }

	}


 void checkCounter(tdCounter *_pCounter)
	{
	if( _pCounter->pNode == NULL )
		return;

	_pCounter->pNode->value = checkLeafs( _pCounter->pNode );
	//checkLeafs2( _pCounter->pNode );


	if( _pCounter->pNode->value )
	   _pCounter->counter += 1;
	}



tdCounter Counter;
tdNode num23;
tdNode numX;
tdNode check;

tdCounter parsedCounter;
tdNode checkParsedCounter;

void setCounter(const string &_equation)
	{
	parsedCounter.name = "parsedCounter";
	checkParsedCounter.pLeft = NULL;
	checkParsedCounter.pRight = NULL;
	parsedCounter.pNode = &checkParsedCounter;

	parser(_equation, parsedCounter.pNode);
	if( parsedCounter.pNode == NULL )
		cout<<"LAL"<<endl;
	}


void fillCounter(void)
	{
	num23.name = "num23";
	num23.type = tdNodeType::value;
	num23.value = 1;
	num23.pLeft = NULL;
	num23.pRight = NULL;

	numX.name = "numX";
	numX.type = tdNodeType::value;
	numX.value = 23;
	numX.pLeft = NULL;
	numX.pRight = NULL;

	check.name = "check";
	check.type = tdNodeType::plas;
	check.pLeft = &num23;
	check.pRight = &numX;

	Counter.name = "Counter 1";
	Counter.counter = 0;
	Counter.pNode = &check;

    }

void printCounterValue(tdCounter *_pCounter)
	{
	cout<<_pCounter->name.c_str()<<endl;
	cout<<"pCounter->pNode->value == "<<_pCounter->pNode->value<<endl;
	cout<<"Counter == "<<_pCounter->counter<<endl;
	cout<<endl;
	}


void test(void)
	{
	fillCounter();
	checkCounter(&Counter);
	printCounterValue(&Counter);

	setCounter(" 13 + 6 ");
	checkCounter(&parsedCounter);
	printCounterValue(&parsedCounter);
	};

int _tmain(int argc, _TCHAR* argv[])
	{
	test();
	getchar();
	return 0;
	}*/
