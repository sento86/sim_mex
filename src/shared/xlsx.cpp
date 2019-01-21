
#define MINIZ_HEADER_FILE_ONLY
#include "miniz/miniz.c" 

#include "xlsx.hpp"



namespace xlsx
{

	Strings * ParseSharedStrings( const char *data )
	{
		const char *p = data;
		int num_strings = 0;
		int buffer_size = 0;
		while( true ) {
			const char *str = strstr( p, "<t>"  );
			const char *end = strstr( p, "</t>" );			
			if( !str || end <= str ) break;
			buffer_size += end - (str+3) + 1;
			num_strings += 1;
			p = end + 4;
		}

		Strings *strings = (Strings*) std::malloc( sizeof(Strings) + num_strings*sizeof(int) + buffer_size );
		strings->num_strings = num_strings;
		
		int  *offset = (int*) ( strings + 1 );
		char *text   = (char*) ( offset + num_strings );
		while( true ) {
			const char *str = strstr( data, "<t>"  );
			const char *end = strstr( data, "</t>" );
			if( !str || end <= str ) break;
			*offset++ = text - (char*)( strings + 1 );
			const int len = end - (str+3);
			strncpy( text, (str+3), len );
			text[len] = '\0';
			text += len + 1;
			data = end + 4;
		}
		
		return strings;
	}
	

	class TableParser
	{
		public:
		
			TableParser( TableReaderListener &reader ) : reader(reader), data(0)  {
			}

			void Parse( const char *data ) {
				this->data = data;
				this->ParseTable();
			}
			
		private:

			bool ParseCell( int row, int col )
			{
				data = strchr( data, '<' );
				ASSERT( data, "ParseCell: Expected tag symbol '<' for 'c'" );
				if( data[1] != 'c' ) return false;
				data += 2;

				while( *data && *data != '>' ) data++;
				if( data[-1] == '/' ) {
					this->reader.OnValue( row, col, 0, NULL );
					return true;
				}
				
				const char *v0 = strstr( data, "<v>" );
				ASSERT( v0, "ParseCell: Expected open tag 'v'" );
				
				const char *v1 = strstr( v0, "</v>" );
				ASSERT( v1, "ParseCell: Expected close tag 'v'" );
				data = v1 + 4;

				v0 += 3;
				while( v0 < v1 && *v0 <= ' ' ) v0++;
				while( v0 < v1 && *v1 <= ' ' ) v1--;
				
				this->reader.OnValue( row, col, v1-v0, v0 );

				data = strstr( data, "</c>" );
				ASSERT( data, "ParseCell: Expected close tag 'c'" );
				data += 4;
				
				return true;
			}

			bool ParseRow( int row )
			{
				data = strchr( data, '<' );
				ASSERT( data, "ParseRow: Expected tag symbol '<' for 'row'" );
				if( strncmp( data, "<row", 4 ) ) return false;
				data += 4;
				
				this->reader.OnRow( row );
				
				bool ok = true;
				for( int col = 0; ok; col++ )
					ok = ParseCell( row, col );
				
				data = strstr( data, "</row>" );
				ASSERT( data, "ParseRow: Expected close tag 'row'" );
				data += 6;
				
				return true;
			}

			void ParseTable( void )
			{
				data = strstr( data, "<sheetData>" );
				ASSERT( data, "ParseTable: Expected open tag 'sheetData'" );
				data += 11;
				
				this->reader.OnBegin();
				
				bool ok = true;
				for( int row = 0; ok; row++ )
					ok = ParseRow( row );

				data = strstr( data, "</sheetData>" );
				ASSERT( data, "ParseTable: Expected close tag 'sheetData'" );

				this->reader.OnEnd();
			}

		private:
		
			TableReaderListener			&reader;
			const char					*data;
	};


}  // namespace xlsx




	
xlsx::File::File( const char *filename )
{
	mz_zip_archive *zip = (mz_zip_archive*) std::malloc( sizeof(mz_zip_archive) );
	ASSERT( zip, "xlsx::File::Open: Out of memory" );

	std::memset( zip, 0, sizeof(*zip) );

	this->pimp   = (void*) zip;
	this->opened = false;
	
	if( filename ) this->Open( filename );
}

xlsx::File::~File()
{
	this->Close();

	if( this->pimp )
		std::free( this->pimp );
}

void xlsx::File::Open( const char *filename )
{
	mz_zip_archive *zip = (mz_zip_archive*) this->pimp;	
	DBG_ASSERT( zip );
	
	if( this->opened ) this->Close();
	
	bool ok = mz_zip_reader_init_file( zip, filename, 0 );
	if( !ok ) print::Error( "xlsx::File::Open: Can not open file '%s'", filename );
	
	this->opened = true;
}


void xlsx::File::Close( void )
{
	mz_zip_archive *zip = (mz_zip_archive*) this->pimp;	
	DBG_ASSERT( zip );

	if( !this->opened ) return;
	
	bool ok = mz_zip_reader_end( zip );
	ASSERT( ok, "xlsx::File::Close: Can not close XLSX file" );
	
	std::memset( zip, 0, sizeof(*zip) );
	this->opened = false;
}


xlsx::Strings * xlsx::File::AllocStrings( void )
{
	mz_zip_archive *zip = (mz_zip_archive*) this->pimp;	
	DBG_ASSERT( zip );

	size_t size = 0;
	char *data = (char*) mz_zip_reader_extract_file_to_heap( zip, "xl/sharedStrings.xml", &size, 0 );
	if( !data || size <= 0 ) print::Error( "xlsx::File::AllocStrings: Can not read shared strings" );
	data[ size - 1 ] = '\0';

	xlsx::Strings *strings = ParseSharedStrings( data );
	
	::free( data );
	
	return strings;
}


void xlsx::File::ReadTable( int sheet, TableReaderListener &reader )
{
	mz_zip_archive *zip = (mz_zip_archive*) this->pimp;	
	DBG_ASSERT( zip && sheet > 0 );

	char buff[64];		
	sprintf( buff, "xl/worksheets/sheet%d.xml", sheet );
	
	size_t size = 0;
	char *data = (char*) mz_zip_reader_extract_file_to_heap( zip, buff, &size, 0 );
	if( !data || size <= 0 ) print::Error( "xlsx::File::ReadTable: Can not find sheet%d", sheet );
	data[ size - 1 ] = '\0';

	TableParser parser( reader );
	parser.Parse( data );
	
	::free( data );
}

