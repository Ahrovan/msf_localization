
#include "msf_localization_core/block_matrix.h"


namespace BlockMatrix
{

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

    // Check blocks
    if(prod1.getColsSize() != prod2.getRowsSize())
        throw;

    // Multiplication
    for(int i=0; i<prod1.rows(); i++)
    {
        for(int j=0; j<prod2.cols(); j++)
        {
            // Sizes
            int row_block=prod1.getRowsSize(i);
            int col_block=prod2.getColsSize(j);


            // Resize block and set zero
            product_matrix(i,j).resize(row_block, col_block);
            product_matrix(i,j).setZero();

            // Avoid zero multiplication
            if(row_block <= 0 || col_block <= 0)
                continue;

            // Multiplication
            for(int k=0; k<prod1.cols(); k++)
            {
                if(prod1(i,k).cols() == row_block && prod2(k,j).rows() == col_block)
                    product_matrix(i,j)+= prod1(i,k)*prod2(k,j);

            }
        }
    }

    // Analyse the result
    if(product_matrix.analyse())
        throw;


    // End
    return product_matrix;
}


//template<>
//MatrixSparse operator*(const MatrixSparse &prod1, const MatrixSparse &prod2);


/*
MatrixSparse operator+(MatrixSparse &sum1, MatrixSparse &sum2)
{
    //Matrix<TypeR> product_matrix;
    MatrixSparse product_matrix;

    // Check multiplication sizes
    if(sum1.cols() != sum2.cols() || sum1.rows() != sum2.rows() )
        throw;

    // Check block sizes
    if(sum1.analyse())
        throw;
    if(sum2.analyse())
        throw;

    if(sum1.getColsSize() != sum2.getColsSize())
        throw;
    if(sum1.getRowsSize() != sum2.getRowsSize())
        throw;

    // Resize
    product_matrix.resize(sum1.rows(), sum1.cols());

    // Addition
    for(int i=0; i<sum1.rows(); i++)
    {
        for(int j=0; j<sum1.cols(); j++)
        {
            // sum1 has no block, sum 2 has block
            if( ( sum1(i,j).cols() == 0 || sum1(i,j).rows() == 0 ) &&
                    ( sum2(i,j).cols() != 0 && sum2(i,j).rows() != 0 ) )
            {
                product_matrix(i,j).resize(sum2(i,j).cols(), sum2(i,j).rows());
                product_matrix(i,j)=sum2(i,j);
            }

            // sum1 has block, sum 2 has no block
            if( ( sum1(i,j).cols() != 0 && sum1(i,j).rows() != 0 ) &&
                    ( sum2(i,j).cols() == 0 || sum2(i,j).rows() == 0 ) )
            {
                product_matrix(i,j).resize(sum1(i,j).cols(), sum1(i,j).rows());
                product_matrix(i,j)=sum1(i,j);
            }

            // sum1 has block, sum 2 has block
            if( ( sum1(i,j).cols() != 0 && sum1(i,j).rows() != 0 ) &&
                    ( sum2(i,j).cols() != 0 && sum2(i,j).rows() != 0 ) )
            {
                // Add
                product_matrix(i,j).resize(sum1(i,j).cols(), sum1(i,j).rows());
                product_matrix(i,j)=sum1(i,j)+sum2(i,j);
            }

            // sum1 has no block, sum 2 has no block
            if( ( sum1(i,j).cols() == 0 || sum1(i,j).rows() == 0 ) &&
                    ( sum2(i,j).cols() == 0 || sum2(i,j).rows() == 0 ) )
            {
                product_matrix(i,j).resize(0, 0);
            }
        }
    }

    // End
    return product_matrix;
}
*/



Eigen::SparseMatrix<double> convertToEigenSparse(const MatrixSparse& in)
{
    Eigen::SparseMatrix<double> out;

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

    // Second pass to get the non-zero coeffs in the appropiate coordinate
    std::vector<Eigen::Triplet<double>> triplet_list;
    triplet_list.reserve(size_non_zero_coef);
    int row_point=0;
    for(int i=0; i<in.rows(); i++)
    {
        int col_point=0;
        for(int j=0; j<in.cols(); j++)
        {
            for (int k=0; k<in(i,j).outerSize(); ++k)
            {
                for (Eigen::SparseMatrix<double>::InnerIterator it(in(i,j),k); it; ++it)
                {
                    triplet_list.push_back(Eigen::Triplet<double>(it.row()+row_point,it.col()+col_point,it.value()));
                }
            }
            col_point+=in.getColsSize(j);
        }
        row_point+=in.getRowsSize(i);
    }

    // Set out
    out.setFromTriplets(triplet_list.begin(), triplet_list.end());

    // End
    return out;
}

Eigen::MatrixXd convertToEigenDense(const MatrixSparse &in)
{
    Eigen::MatrixXd out;


    // Resize and init out
    out.resize(in.getTotalRowsSize(), in.getTotalColsSize());
    out.setZero();

    // Fill blocks
    int row_point=0;
    for(int i=0; i<in.rows(); i++)
    {
        int col_point=0;
        for(int j=0; j<in.cols(); j++)
        {
            // Block
            if(in(i,j).cols() > 0 && in(i,j).rows() > 0 )
                out.block(row_point, col_point, in.getRowsSize(i), in.getColsSize(j))=Eigen::MatrixXd(in(i,j));

            col_point+=in.getColsSize(j);
        }
        row_point+=in.getRowsSize(i);
    }

    // End
    return out;
}

Eigen::MatrixXd convertToEigenDense(const MatrixDense &in)
{
    Eigen::MatrixXd out;

    // Resize and init out
    out.resize(in.getTotalRowsSize(), in.getTotalColsSize());
    out.setZero();

    // Fill blocks
    int row_point=0;
    for(int i=0; i<in.rows(); i++)
    {
        int col_point=0;
        for(int j=0; j<in.cols(); j++)
        {
            // Block
            if(in(i,j).cols() > 0 && in(i,j).rows() > 0 )
                out.block(row_point, col_point, in.getRowsSize(i), in.getColsSize(j))=in(i,j);

            col_point+=in.getColsSize(j);
        }
        row_point+=in.getRowsSize(i);
    }

    // End
    return out;
}


std::vector< Eigen::Triplet<double> > getVectorEigenTripletFromEigenDense(const Eigen::MatrixXd& in, int row_offset, int col_offset)
{
    std::vector< Eigen::Triplet<double> > triplets;
    for(int i=0; i<in.rows(); i++)
        for(int j=0; j<in.cols(); j++)
            triplets.push_back(Eigen::Triplet<double>(i+row_offset,j+col_offset,in(i,j)));

    return triplets;
}

int insertVectorEigenTripletFromEigenDense(std::vector< Eigen::Triplet<double> >& triplets, const Eigen::MatrixXd& in, int row_offset, int col_offset)
{
    for(int i=0; i<in.rows(); i++)
        for(int j=0; j<in.cols(); j++)
            if( std::abs(in(i,j)) > std::numeric_limits<double>::epsilon() ) // More efficient than a prune!
                triplets.push_back(Eigen::Triplet<double>(i+row_offset, j+col_offset, in(i,j)));
    return 0;
}



}
