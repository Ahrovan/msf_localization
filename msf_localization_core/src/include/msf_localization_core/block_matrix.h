
#ifndef _BLOCK_MATRIX_H
#define _BLOCK_MATRIX_H


#include <Eigen/Dense>
#include <Eigen/Sparse>


namespace BlockMatrix
{

// Types
template<class Type>
class Matrix : public Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>
{
protected:
    Eigen::VectorXi rows_size_;
    Eigen::VectorXi cols_size_;

public:
    Eigen::VectorXi getRowsSize() const
    {
        return rows_size_;
    }
    Eigen::VectorXi getColsSize() const
    {
        return cols_size_;
    }

    int getTotalRowsSize() const
    {
        int total_rows_size=0;

        for(int i=0; i<rows_size_.rows(); i++)
            total_rows_size+=rows_size_(i);

        return total_rows_size;
    }
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
        // Find dimensions
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

        // Resize each block
        for(int row=0; row<num_rows; row++)
        {
            for(int col=0; col<num_cols; col++)
            {
                this->operator()(row, col).resize(0, 0);
            }
        }

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
            for(int row=0; row<this->rows(); row++)
            {
                for(int col=0; col<this->cols(); col++)
                {
                    this->operator()(row,col)=other(row,col);
                }
            }
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
        for(int row=0; row<this->rows(); row++)
            for(int col=0; col<this->cols(); col++)
            {
                if(this->operator()(row, col).cols()>0 && this->operator()(row, col).rows()>0)
                {
                    // Resize block
                    out(col,row).resize(this->operator()(row, col).cols(), this->operator()(row, col).rows());
                    // Set block transposed
                    out(col,row)=this->operator()(row, col).transpose();
                }
            }

        return out;
    }

    /*
    bool operator()()
    {


        return true;
    }
    */


    Matrix operator+(Matrix& sum1)
    {
        Matrix sum_result;

        // Check big sizes
        if(sum1.cols() != this->cols())
            throw;
        if(sum1.rows() != this->rows())
            throw;

        // Check blocks sizes
        // Analyse both
        if(this->analyse())
            throw;
        if(sum1.analyse())
            throw;
        //Check
        if(this->getColsSize() != sum1.getColsSize())
            throw;
        if(this->getRowsSize() != sum1.getRowsSize())
            throw;

        // Everything is fine.

        // Resize the result
        sum_result.resize(this->rows(), this->cols());

        // Sum the blocks
        for(int i=0; i<this->rows(); i++)
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

        // End
        return sum_result;
    }



};


using MatrixSparse = Matrix<Eigen::SparseMatrix<double>>;

using MatrixDense = Matrix<Eigen::MatrixXd>;




MatrixSparse operator*(const MatrixSparse &prod1, const MatrixSparse &prod2);

Eigen::SparseMatrix<double> convertToEigenSparse(MatrixSparse& in);

Eigen::MatrixXd convertToEigenDense(const MatrixSparse& in);


std::vector< Eigen::Triplet<double> > getVectorEigenTripletFromEigenDense(const Eigen::MatrixXd& in, int row_offset=0, int col_offset=0);

int insertVectorEigenTripletFromEigenDense(std::vector< Eigen::Triplet<double> >& triplet, const Eigen::MatrixXd& in, int row_offset=0, int col_offset=0);




}







#endif
