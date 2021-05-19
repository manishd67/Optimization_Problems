#include<iostream>
#include<fstream>
#include<ilcplex/ilocplex.h>
#include<vector>
#include<string>

using namespace std;

typedef IloArray<IloNumArray> Matrix;	// for transportation costs
typedef IloArray<IloNumVarArray> variables;	// for quantity transproted


int main() {
	IloEnv env;

	// Reading input
	ifstream input;
	string filename = "cap71.txt";
	string folder_path = "Add the folder path for the input file here";
	input.open( folder_path + filename);
	int n, m;  // n = no. of potential facility locations, m = no. of clients
	input >> n >> m;
	IloNumArray fixed_costs(env, n);
	IloNumArray demands(env, m);
	Matrix tp_costs(env, n);	// Matrix for transprotation costs where each entry represents 
	//the cost of meeting entire demand of jth client by ith facility

	string temp;
	

	for (int i = 0; i < n; i++) {
		input >> temp;		//ignored capacity term
		input >> fixed_costs[i];
		tp_costs[i] = IloNumArray(env, m);
	}

	for (int j = 0; j < m; j++) {
		input >> demands[j];
		for (int i = 0; i < n; i++) {
			input >> tp_costs[i][j];
		}
	}
	input.close();

	IloNum D = 0;
	for (int j = 0; j < m; j++) {
		D += demands[j];
	}


	//Solution
	IloModel UFLP(env);
	IloNumVarArray x(env, n, 0, 1, ILOINT);
	variables z(env, n);
	for (int i = 0; i < n; i++) {
		z[i] = IloNumVarArray(env, m, 0, 1, ILOINT);
	}

	//Objective function
	IloExpr fc(env), tc(env); // fc = fixed cost, tc = transprotation costs
	for (int i = 0; i < n; i++) {
		fc += fixed_costs[i] * x[i];
		for (int j = 0; j < m; j++) {
			tc += tp_costs[i][j] * z[i][j];
		}
	}

	IloObjective obj = IloMinimize(env, fc + tc);
	UFLP.add(obj);

	//Contraints

	for (int j = 0; j < m; j++) {
		IloExpr exp(env);
		for (int i = 0; i < n; i++) {
			exp += z[i][j];
		}
		UFLP.add(exp == 1);
	}

	for (int i = 0; i < n; i++) {
		IloExpr exp1(env);
		for (int j = 0; j < m; j++) {
			exp1 += z[i][j];
		}
		UFLP.add(exp1 <=  m*x[i]);
	}

	IloCplex cplex(UFLP);
	//cplex.setOut(env.getNullStream());
	cplex.solve();
	IloNumArray plants(env);

	cplex.getValues(plants, x);
	cout << "Facility Locations\t = "<< plants << endl;
	cout << endl<< "Optimal_solution\t"<<cplex.getObjValue() << endl;

	
	/* Connections matrix - 1 in matrix represents
	client j connected to facility i */
	cout << endl << "1 in matrix represents client j connected to facility i" << endl;
	Matrix links(env, n);
	IloNumArray res(env,m);
	for (int i = 0; i < n; i++) {
		links[i] = IloNumArray(env);
		cplex.getValues(links[i], z[i]);
		for (int j = 0; j < m; j++) {
			if (links[i][j] != 1)
				links[i][j] = 0;
			else
				res[j] = i + 1;
		}
		cout << "[ ";
		for (int j = 0; j < m; j++) {
			cout << links[i][j]<<" ";
		}
		cout << "]" << endl;
	}


	// Printing results to txt file
	ofstream output;
	output.open("Add the file path for the output.txt file here", std::ios::app);
	output << filename << endl;
	output << "Optimal_solution - " << cplex.getObjValue() << endl;
	output << "j th client connected to ith facility. i = 1,2,...n" << endl;
	output << res << endl<<endl;
	output.close();


	env.end();
	return 0;
}
