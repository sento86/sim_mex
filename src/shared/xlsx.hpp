
#ifndef __XLSX_HPP__
#define __XLSX_HPP__


#include <cstring>
#include <vector>

#include "main.hpp"



namespace xlsx
{

	struct TableReaderListener {
		virtual void OnBegin( void )  { };
		virtual void OnEnd( void )    { };
		virtual void OnRow( int row ) { };
		virtual void OnValue( int row, int col, int size, const char *value ) = 0;
	};
	
	
	template < typename Row >
	struct TableReader : TableReaderListener
	{
		TableReader( std::vector<Row> &table ) : table(table) { }

		void OnValue( int row, int col, int size, const char *data ) {
			if( col == 0 ) this->table.emplace_back();
			this->table.back().SetValue( col, size, data );
		}
		
		private:
		
			std::vector<Row> &table;
	};

	
	struct Strings
	{
		int num_strings = 0;
		
		inline const char * Get( int index ) const {
			DBG_ASSERT( index >= 0 && index < num_strings );
			const int *data = (int*) ( this + 1 );
			return (char*)data + data[index];
		}
		
		inline const char * Get( const char *index ) const {
			return this->Get( std::atoi(index) );
		}
	};


	class File
	{
		public:
		
			File( const char *filename=NULL );			
			virtual ~File();
			
			void Open( const char *filename );
			void Close( void );

			Strings * AllocStrings( void );

			void ReadTable( int sheet, TableReaderListener &reader );
			
			template < typename Row >
			void ReadTable( int sheet, std::vector<Row> &table ) {
				table.clear();
				TableReader<Row> reader( table );
				this->ReadTable( sheet, reader );
			}

			template < typename Row >
			std::vector<Row> ReadTable( int sheet ) {
				std::vector<Row> table;
				TableReader<Row> reader( table );
				this->ReadTable( sheet, reader );
				return table;
			}
			
		private:
					
			void  *pimp;
			bool  opened;
	};


}  // namespace xlsx



#endif  // __XLSX_HPP__
