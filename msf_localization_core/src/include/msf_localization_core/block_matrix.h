
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



// Operators
//template<class Type1, class Type2, class TypeR>
//Matrix<TypeR> operator*(const Matrix<Type1> &prod1, const Matrix<Type2> &prod2)
MatrixSparse operator*(const MatrixSparse &prod1, const MatrixSparse &prod2)
{
    //Matrix<TypeR> product_matrix;
    MatrixSparse product_matrix;

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


//template<>
//MatrixSparse operator*(const MatrixSparse &prod1, const MatrixSparse &prod2);



Eigen::SparseMatrix<double> convertToEigenSparse(MatrixSparse& in)
{
    Eigen::SparseMatrix<double> out;

    // Analyse
    if(in.analyse())
        throw;

    // Resize out
    out.resize(in.getTotalRowsSize(), in.getTotalColsSize());

    //std::cout<<"rows="<<out.rows()<<"; cols="<<out.cols()<<std::endl;

    // First pass to get the size of non-zero coefficient
    int size_non_zero_coef=0;
    for(int i=0; i<in.rows(); i++)
        for(int j=0; j<in.cols(); j++)
            size_non_zero_coef+=in.nonZeros();

    // Reserve out
    out.reserve(size_non_zero_coef);

    //std::cout<<"size_non_zero_coef="<<size_non_zero_coef<<std::endl;

    // Second pass to get the non-zero coeffs
    std::vector<Eigen::Triplet<double>> triplet_list;
    triplet_list.reserve(size_non_zero_coef);
    for(int i=0; i<in.rows(); i++)
    {
        for(int j=0; j<in.cols(); j++)
        {
            for (int k=0; k<in(i,j).outerSize(); ++k)
            {
                for (Eigen::SparseMatrix<double>::InnerIterator it(in(i,j),k); it; ++it)
                {
                    triplet_list.push_back(Eigen::Triplet<double>(it.row(),it.col(),it.value()));
                }
            }
        }
    }

    // Set out
    out.setFromTriplets(triplet_list.begin(), triplet_list.end());

    // End
    return out;
}


Eigen::MatrixXd convertToEigenDense(const MatrixSparse& in)
{
    Eigen::MatrixXd out;


    // TODO


    return out;
}


}







#endif
