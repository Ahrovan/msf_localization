
#ifndef _BLOCK_MATRIX_H
#define _BLOCK_MATRIX_H


#include <Eigen/Dense>
#include <Eigen/Sparse>


namespace BlockMatrix
{

// Types
template<class Type, int RowsSize, int ColsSize>
class Matrix : public Eigen::Matrix<Type, RowsSize, ColsSize>
{
protected:
    Eigen::VectorXi rows_size_;
    Eigen::VectorXi cols_size_;

public:
    Eigen::VectorXi getRowsSize() const
    {
        return rows_size_;
    }
    int getRowsSize(int row_i) const
    {
        return rows_size_(row_i);
    }
public:
    Eigen::VectorXi getColsSize() const
    {
        return cols_size_;
    }
    int getColsSize(int col_i) const
    {
        return cols_size_(col_i);
    }
public:
    int getTotalRowsSize() const
    {
        int total_rows_size=0;

        for(int i=0; i<rows_size_.rows(); i++)
            total_rows_size+=rows_size_(i);

        return total_rows_size;
    }
public:
    int getTotalColsSize() const
    {
        int total_cols_size=0;

        for(int i=0; i<cols_size_.rows(); i++)
            total_cols_size+=cols_size_(i);

        return total_cols_size;
    }

protected:
public:
    int analyse()
    {
        // Resize and init to zero -> Not needed. Done when resize()
//        rows_size_.resize(this->rows());
//        rows_size_.setZero();
//        cols_size_.resize(this->cols());
//        cols_size_.setZero();


        // special case 1: this->cols() == 0
        if( this->cols() == 0 && this->rows() != 0 )
        {

            return 0;
        }


        // special case 2: this->rows() == 0
        if(this->rows() == 0 && this->cols() != 0)
        {

            return 0;
        }


        // special case 3: rows()==0 && cols()==0
        if(this->rows() == 0 && this->cols() == 0)
        {

            return 0;
        }



        // Normal cases

        // rows
        for(int i=0; i<this->rows(); i++)
        {
            int row_i_dimension=0;
            for(int j=0; j<this->cols(); j++)
            {
                if(this->operator()(i,j).rows() == 0)
                    continue;
                if(row_i_dimension == 0)
                {
                    row_i_dimension=this->operator()(i,j).rows();
                }
                if(row_i_dimension != this->operator()(i,j).rows())
                {
                    return -1;
                }
            }
            rows_size_(i)=row_i_dimension;
        }

        // cols
        for(int j=0; j<this->cols(); j++)
        {
            int col_j_dimension=0;
            for(int i=0; i<this->rows(); i++)
            {
                if(this->operator()(i,j).cols() == 0)
                    continue;
                if(col_j_dimension == 0)
                {
                    col_j_dimension=this->operator()(i,j).cols();
                }
                if(col_j_dimension != this->operator()(i,j).cols())
                {
                    return -1;
                }
            }
            cols_size_(j)=col_j_dimension;
        }

        // End
        return 0;
    }

public:
    int createFromEigen(const Type& mat_in, const Eigen::VectorXi& rows_size, const Eigen::VectorXi& cols_size)
    {
        // Resize
        this->resize(rows_size, cols_size);

        // Checks
        if( getTotalRowsSize() != mat_in.rows() || getTotalColsSize() != mat_in.cols() )
            throw;

        // Fill blocks
        int num_rows=rows_size.rows();
        int num_cols=cols_size.rows();
        int dimension_row_i=0;
        for(int row=0; row<num_rows; row++)
        {
            int dimension_col_i=0;
            for(int col=0; col<num_cols; col++)
            {
                this->operator()(row, col)=mat_in.block(dimension_row_i, dimension_col_i, rows_size_(row, 0), cols_size_(col, 0));
                dimension_col_i+=cols_size_(col, 0);
            }
            dimension_row_i+=rows_size_(row, 0);
        }

        // Analyse
        if(this->analyse())
            throw;

        // End
        return 0;
    }

    int createFromEigen(const Type& mat_in, const Eigen::VectorXi& rows_size)
    {
        Eigen::VectorXi cols_size(1);
        cols_size(0)=1;

        int error=createFromEigen(mat_in, rows_size, cols_size);
        if(error)
            return error;

        // End
        return 0;
    }

    void resize(int num_rows, int num_cols)
    {
        if(num_rows < 0 || num_cols < 0)
            throw;

        // Resize Block Data Matrix
        Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>::resize(num_rows, num_cols);

        // Resize vectors for rows_size and cols_size
        rows_size_.resize(num_rows);
        cols_size_.resize(num_cols);

        // Set zero
        rows_size_.setZero();
        cols_size_.setZero();

        // Resize each block -> Not Needed?
        /*
        for(int row=0; row<num_rows; row++)
        {
            for(int col=0; col<num_cols; col++)
            {
                this->operator()(row, col).resize(0, 0);
            }
        }
        */

        return;
    }

    void resize(const Eigen::VectorXi& rows_size, const Eigen::VectorXi& cols_size)
    {
        // Checks
        if(rows_size.rows() == 0 || cols_size.rows() == 0)
            throw;

        // Reset
        rows_size_=rows_size;
        cols_size_=cols_size;

        // Resize Block Data Matrix
        int num_rows=rows_size_.rows();
        int num_cols=cols_size_.rows();
        Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>::resize(num_rows, num_cols);

        // Resize each blocks
        #pragma omp parallel for
        for(int row=0; row<num_rows; row++)
        {
            #pragma omp parallel for
            for(int col=0; col<num_cols; col++)
            {
                this->operator()(row, col).resize(rows_size_(row,0), cols_size_(col,0));
            }
        }

        // End
        return;
    }

    Matrix& operator=(const Matrix& other)
    {
        // protect against invalid self-assignment
        if (this != &other)
        {
            // Resize
            this->resize(other.rows(), other.cols());
            // Copy Values of each block
            #pragma omp parallel for
            for(int row=0; row<this->rows(); row++)
            {
                #pragma omp parallel for
                for(int col=0; col<this->cols(); col++)
                {
                    this->operator()(row,col)=other(row,col);
                }
            }
            // Analyse
            if(this->analyse())
                throw;
        }
        // by convention, always return *this
        return *this;
    }


    Matrix transpose()
    {
        Matrix out;

        // Resize
        out.resize(this->cols(), this->rows());
        // Copy Values of each block
        #pragma omp parallel for
        for(int row=0; row<this->rows(); row++)
        {
            #pragma omp parallel for
            for(int col=0; col<this->cols(); col++)
            {
                // Resize block
                out(col,row).resize(this->operator()(row, col).cols(), this->operator()(row, col).rows());
                // Set block transposed
                if(this->operator()(row, col).cols()>0 && this->operator()(row, col).rows()>0)
                {
                    out(col,row)=this->operator()(row, col).transpose();
                }
            }
        }
        // Analyse out
        if(out.analyse())
            throw;

        return out;
    }

    /*
    bool operator()()
    {


        return true;
    }
    */


    Matrix operator+(const Matrix& sum1)
    {
        Matrix sum_result;

        // Check big sizes
        if(sum1.cols() != this->cols())
            throw;
        if(sum1.rows() != this->rows())
            throw;

        // Check blocks sizes
        //Check
        if(this->getColsSize() != sum1.getColsSize())
            throw;
        if(this->getRowsSize() != sum1.getRowsSize())
            throw;

        // Everything is fine.

        // Resize the result
        sum_result.resize(this->rows(), this->cols());

        // Sum the blocks
        #pragma omp parallel for
        for(int i=0; i<this->rows(); i++)
        {
            #pragma omp parallel for
            for(int j=0; j<this->cols(); j++)
            {
                // Two blocks non zero
                if( ( this->operator()(i,j).cols() != 0 && this->operator()(i,j).rows() != 0 )
                        && ( sum1(i,j).cols() != 0 && sum1(i,j).rows() != 0 ) )
                {
                    // Resize
                    sum_result(i,j).resize(sum1(i,j).rows(), sum1(i,j).cols());

                    // Sum
                    sum_result(i,j)=sum1(i,j)+this->operator()(i,j);

                    continue;
                }
                // this(i,j) block zero
                else if( ( this->operator()(i,j).cols() == 0 || this->operator()(i,j).rows() == 0 )
                         && ( sum1(i,j).cols() != 0 && sum1(i,j).rows() != 0 ) )
                {
                    // Resize
                    sum_result(i,j).resize(sum1(i,j).rows(), sum1(i,j).cols());

                    // Sum
                    sum_result(i,j)=sum1(i,j);
                }
                // sum1(i,j) block zero
                else if( ( this->operator()(i,j).cols() != 0 && this->operator()(i,j).rows() != 0 )
                         && ( sum1(i,j).cols() == 0 || sum1(i,j).rows() == 0 ) )
                {
                    // Resize
                    sum_result(i,j).resize(this->operator()(i,j).rows(), this->operator()(i,j).cols());

                    // Sum
                    sum_result(i,j)=this->operator()(i,j);
                }
                // The two blocks zero
                else
                {
                    // Do nothing
                    continue;
                }


            }
        }

        // Analyse the result
        if(sum_result.analyse())
            throw;

        // End
        return sum_result;
    }


    Matrix operator+=(const Matrix& sum1)
    {
        //
        Matrix sum_result;

        // Operator+()
        sum_result=this->operator+(sum1);

        // End
        return sum_result;
    }



};


using MatrixSparse = Matrix<Eigen::SparseMatrix<double>, Eigen::Dynamic, Eigen::Dynamic>;
/*
class MatrixSparse : public Matrix<Eigen::SparseMatrix<double>, Eigen::Dynamic, Eigen::Dynamic>
{

};
*/



using MatrixDense = Matrix<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Dynamic, Eigen::Dynamic>;
/*
class MatrixDense : public Matrix<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic>
{

};


class VectorSparse : public Matrix<Eigen::SparseMatrix<double>, Eigen::Dynamic, 1>
{

};

class VectorDense : public Matrix<Eigen::MatrixXd, Eigen::Dynamic, 1>
{

};
*/


MatrixSparse operator*(const MatrixSparse &prod1, const MatrixSparse &prod2);

MatrixDense operator*(const MatrixDense &prod1, const MatrixSparse &prod2);
MatrixDense operator*(const MatrixSparse &prod1, const MatrixDense &prod2);


//MatrixSparse operator+(MatrixSparse &sum1, MatrixSparse &sum2);
MatrixDense operator+(const MatrixDense &sum1, const MatrixSparse &sum2);
MatrixDense operator+(const MatrixSparse &sum1, const MatrixDense &sum2);

/*
template<class TypeResult, class TypeProd1, class TypeProd2>
TypeResult operator*(const TypeProd1 &prod1, const TypeProd2 &prod2)
{
    //Matrix<TypeR> product_matrix;
    TypeResult product_matrix;

    // Check multiplication sizes
    if(prod1.cols() != prod2.rows())
        throw;

    // Resize
    product_matrix.resize(prod1.rows(), prod2.cols());

    // Multiplication
    for(int i=0; i<prod1.rows(); i++)
        for(int j=0; j<prod2.cols(); j++)
        {
            // First pass -> checks and dimension of the block
            int row_block=-1;
            int col_block=-1;
            for(int k=0; k<prod1.cols(); k++)
            {
                // Check prod1
                if(prod1(i,k).cols() == 0 || prod1(k,j).rows() == 0)
                    continue;

                // Check prod2
                if(prod2(i,k).cols() == 0 || prod2(k,j).rows() == 0)
                    continue;

                // Check prod1 * prod2
                if(prod1(i,k).cols() != prod2(k,j).rows())
                    throw;

                // Rows and cols
                if(row_block<0)
                    row_block=prod1(k,j).rows();
                if(col_block<0)
                    col_block=prod2(k,j).cols();

                // Check
                if(row_block != prod1(k,j).rows())
                    throw;

                if(col_block != prod2(k,j).cols())
                    throw;
            }

            // Dimensions
            if(row_block < 0 || col_block < 0)
                continue;

            // Resize and set zero
            product_matrix(i,j).resize(row_block, col_block);
            product_matrix(i,j).setZero();

            // Second pass -> Multiplication
            for(int k=0; k<prod1.cols(); k++)
            {
                if(prod1(i,k).cols() == row_block && prod2(k,j).rows() == col_block)
                    product_matrix(i,j)+= prod1(i,k)*prod2(k,j);

            }
        }


    // End
    return product_matrix;
}


// Template specialization
template<>
MatrixSparse operator*(const MatrixSparse &prod1, const MatrixSparse &prod2);
*/


Eigen::SparseMatrix<double> convertToEigenSparse(const MatrixSparse &in);
// TODO
//Eigen::MatrixXd convertToEigenDense(MatrixSparse& in);


Eigen::MatrixXd convertToEigenDense(const MatrixSparse &in);
Eigen::MatrixXd convertToEigenDense(const MatrixDense &in);


std::vector< Eigen::Triplet<double> > getVectorEigenTripletFromEigenDense(const Eigen::MatrixXd& in, int row_offset=0, int col_offset=0);

int insertVectorEigenTripletFromEigenDense(std::vector< Eigen::Triplet<double> >& triplet, const Eigen::MatrixXd& in, int row_offset=0, int col_offset=0);




}







#endif
