#pragma once
#include <vector>

/*!
A static class to print to a file.
*/

class FileOut{
public:
	//! Output function for 1d vector, list, etc: anything that can be iterated over
	template<class T>
	static void print_vector(std::vector<T> &output, std::string file_name, std::string separator=","){
		std::ofstream file(file_name.c_str());
		if (file.is_open()){
			print_vector(output, file,separator);
			file.close();
			printf("... Successfully wrote to file %s.\n",file_name.c_str());
		}
		else FileErrors::failed_file(file_name);
	}

	//! Output function for 2d vector, list, etc: anything that can be iterated over
	template<class T>
	static void print_vector(std::vector<std::vector<T> > &output, std::string file_name, std::string separator=","){
		std::ofstream file(file_name.c_str());
		if (file.is_open()){
			for (std::vector<T> outer: output){
				print_vector(outer, file, separator);
				file << "\n";
			}
			file.close();
			printf("... Successfully wrote to file %s.\n",file_name.c_str());
		}
		else FileErrors::failed_file(file_name);
	}


	//! Output function for 3d vector, list, etc: anything that can be iterated over
	template <class T>
	static void print_vector(std::vector<std::vector<std::vector<T> > > &output, std::string file_name, std::string separator=","){
		std::ofstream file(file_name.c_str());
		if (file.is_open()){
			for (std::vector<std::vector<T> > outer: output){
				print_vector(outer, file, separator);
				file << "\n";
			}
			file.close();
			printf("... Successfully wrote to file %s.\n",file_name.c_str());
		}
		else FileErrors::failed_file(file_name);
	}

	//! Print-to-file for a 1D container of pairs (also works with maps).
	template<class T>
	static void print_pair_container(T data, std::string file_name,std::string separator=","){
		std::ofstream file(file_name.c_str());
		if (file.is_open()){
			for (auto it:data){
				file << it.first << separator << it.second << separator << "\n";
			}
			file.close();
			printf("... Successfully wrote to file %s.\n",file_name.c_str());
		} else
			FileErrors::failed_file(file_name);
	}

private:
	//! Helper function for 1- and 2-d file output
	template <class T>
	static void print_vector(std::vector<T> output, std::ofstream &file, std::string separator){
		for (T outer : output){
			file << outer << separator;
		}
	}

	//! Helper function for 3-d file output
	template <class T>
	static void print_vector(std::vector<std::vector<T> > output, std::ofstream &file, std::string separator){
		for (std::vector<T> outer: output){
			print_vector(outer, file, separator);
		}
	}
};