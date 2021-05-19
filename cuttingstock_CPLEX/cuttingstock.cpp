#include<iostream>
#include<fstream>
#include<ilcplex/ilocplex.h>
#include<vector>
#include<string>
using namespace std;

// ADD THE PATHS TO THE INPUT, OUTPUT AND LOG FILES IN LINES 18, 39 AND 88

typedef vector<IloNumArray> Matrix;

//Input function
Matrix read_input(IloNum& width, IloNum& n) {
	IloEnv env1;
	string s;
	Matrix len_demand(2);
	ifstream input;
	input.open("Add path of the input file here");	// eg - "c:\\users\\documents\\input.txt"
	getline(input, s, '\t');
	input >> width;
	getline(input, s, '\t');
	input >> n;
	getline(input, s, ':');
	IloNumArray demand(env1, n);
	IloNumArray len(env1, n);
	for (int i = 0; i < n; i++) {
		input >> len[i] >> demand[i];
	}
	input.close();
	len_demand[0] = len;
	len_demand[1] = demand;
	env1.end();
	return len_demand;
}

//Output function
void write_output(int& stocks_req, double& waste_percent, IloNumArray& len, Matrix& patterns, vector<int> freq) {
	ofstream output;
	output.open("Add path of the output file here", ofstream::out | ofstream::trunc);		//eg - "c:\\users\\documents\\output.txt"
	output << "No. of stocks to be cut: " << stocks_req << endl;

	output << "Waste percentage: " << waste_percent << " %" << endl;
	output << endl << "Order lengths: " << len << endl;
	output << endl << "Cutting Pattern\t\tNo. of times cut" << endl;
	for (int i = 0; i < patterns.size(); i++) {
		output << patterns[i] << "\t\t" << freq[i] << endl;
	}
	output.close();
}


int main() {
	IloEnv env;
	int i, j;
	IloNum width, n;

	// Read Input data
	Matrix len_demand;
	len_demand = read_input(width, n);
	IloNumArray len = len_demand[0];
	IloNumArray demand = len_demand[1];

	//Tolerance
	double tolerance = 1e-6;

	//Master LP
	IloModel cutstock(env);
	IloObjective stocks = IloAdd(cutstock, IloMinimize(env));
	IloNumVarArray x(env); // Decision variables
	//Constraint matrix
	IloRangeArray constr = IloAdd(cutstock, IloRangeArray(env, demand, IloInfinity));
	/* ith constraint constr[i] has lower bound of demand[i]
	and upper bound infinity*/

	Matrix B;  // Initial Basis

	//Constraint matrix with initial patterns
	int k;
	for (i = 0; i < n; i++) {
		IloNumArray temp(env, n);
		k = width / len[i];
		temp[i] = k;
		B.push_back(temp);
		x.add(IloNumVar(stocks(1) + constr[i](k)));
	}

	ofstream log;
	log.open("Add path of the log file here", ofstream::out | ofstream::trunc); //eg - "c:\\users\\documents\\log.txt"
	log << "Initial Patterns added to the master problem" << endl;
	for (i = 0; i < n; i++) {
		log << i << ") " << B[i] << endl;
	}

	IloCplex master(cutstock);
	double reduced_cost = INT32_MAX;
	double current_soln;	// Obj value of master problem
	IloNumArray val(env);	//Decision variable values
	IloNumArray duals(env);	// Dual values

	int iter = 1;		//iteration tracker

	IloModel pat(env);			//Sub IP
	IloNumVarArray a(env, n, 0, IloInfinity, ILOINT); //New column entries
	IloObjective subproblem_obj = IloAdd(pat, IloMaximize(env));	// Subproblem objective

	//feasibity contraint
	pat.add(IloScalProd(len, a) <= width);

	while (true) {
		//master.setOut(env.getNullStream());
		master.solve();

		master.getValues(val, x);		// Solution to master problem
		current_soln = master.getObjValue();

		//Getting dual values
		master.getDuals(duals, constr);

		//Updating the objective function	
		subproblem_obj.setLinearCoefs(a, duals);

		IloCplex sub(pat);
		sub.setOut(env.getNullStream());
		sub.solve();

		IloNumArray newPattern(env); // New Pattern generated
		sub.getValues(newPattern, a);

		// To avoid getting -0 in new generated patterns
		for (i = 0; i < newPattern.getSize(); i++)
		{
			newPattern[i] = abs(newPattern[i]);
		}

		reduced_cost = sub.getObjValue() - 1;

		// Reporting to log file
		log << endl << "iteration " << iter++ << ":" << endl;
		log << "shadow prices/dual values: " << duals << endl;
		log << "objective value of the master-problem: " << current_soln << endl;
		log << "objective value of the sub-problem: " << reduced_cost << endl;


		if (reduced_cost > tolerance) {
			log << "New Pattern Generated: " << newPattern << endl;
			x.add(IloNumVar(stocks(1) + constr(newPattern)));
			B.push_back(newPattern);
		}
		else
			break;
	}

	log << endl << "Optimality attained for the master-problem" << endl;;
	log.close();


	//Patterns used and their frequency
	Matrix patterns;
	vector<int> freq;

	double num;
	for (i = 0; i < B.size(); i++) {
		num = master.getValue(x[i]);
		if (num != 0) {
			patterns.push_back(B[i]);
			freq.push_back(ceil(num));
		}
	}

	//Stocks required
	int stocks_req = 0;
	for (i = 0; i < freq.size(); i++) {
		stocks_req += freq[i];
	}
	// Waste percentage
	double total_length = stocks_req * width;
	double length_req = 0;	//length required
	for (i = 0; i < n; i++) {
		length_req += len[i] * demand[i];
	

	double waste_percent = ((total_length - length_req) * 100) / total_length;

	//Writing to Output file 
	write_output(stocks_req, waste_percent, len, patterns, freq);

	env.end();
	return 0;
}


