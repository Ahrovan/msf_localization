
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <ros/ros.h>


int main(int argc,char **argv)
{

    ros::init(argc, argv, "test");

    ros::NodeHandle n;



    Eigen::MatrixXd MatAdf;
    Eigen::MatrixXd MatBdf;
    Eigen::MatrixXd MatCdf;

    MatAdf.resize(100,100);
    MatAdf.setRandom();

    MatBdf.resize(100,100);
    MatBdf.setRandom();


    Eigen::SparseMatrix<double> MatAsf;
    Eigen::SparseMatrix<double> MatBsf;
    Eigen::SparseMatrix<double> MatCsf;

    MatAsf.resize(100,100);
    MatAsf.reserve(100*100);

    MatBsf.resize(100,100);
    MatBsf.reserve(100*100);

    {
    std::vector<Eigen::Triplet<double>> tripletList;
    tripletList.reserve(100*100);
    for(int i=0;i<100;i++)
    {
      for(int j=0;j<100;j++)
        tripletList.push_back(Eigen::Triplet<double>(i,j,i+j));
    }
    MatAsf.setFromTriplets(tripletList.begin(), tripletList.end());
    MatBsf.setFromTriplets(tripletList.begin(), tripletList.end());
    }




    Eigen::MatrixXd MatAde;
    Eigen::MatrixXd MatBde;
    Eigen::MatrixXd MatCde;

    MatAde.resize(100,100);
    MatAde=Eigen::MatrixXd::Identity(100,100);

    MatBde.resize(100,100);
    MatBde=Eigen::MatrixXd::Identity(100,100);


    Eigen::SparseMatrix<double> MatAse;
    Eigen::SparseMatrix<double> MatBse;
    Eigen::SparseMatrix<double> MatCse;

    MatAse.resize(100,100);
    MatAse.reserve(100);

    MatBse.resize(100,100);
    MatBse.reserve(100);

    {
    std::vector<Eigen::Triplet<double>> tripletList;
    tripletList.reserve(100);
    for(int i=0;i<100;i++)
    {
      for(int j=0;j<100;j++)
          if(i==j)
        tripletList.push_back(Eigen::Triplet<double>(i,j,1));
    }
    MatAse.setFromTriplets(tripletList.begin(), tripletList.end());
    MatBse.setFromTriplets(tripletList.begin(), tripletList.end());
    }




    {
        std::cout<<"Test 01: df x df"<<std::endl;
        ros::Time time_before=ros::Time::now();
        for(int i=0; i<100; i++)
        {

            MatCdf=MatAdf*MatBdf;
        }
        ros::Time time_after=ros::Time::now();

        std::cout<<"\t delta time="<<(time_after-time_before)<<" ns"<<std::endl;
    }

    {
        std::cout<<"Test 02: sf x sf"<<std::endl;
        ros::Time time_before=ros::Time::now();
        for(int i=0; i<100; i++)
        {

            MatCsf=MatAsf*MatBsf;
        }
        ros::Time time_after=ros::Time::now();

        std::cout<<"\t delta time="<<(time_after-time_before)<<" ns"<<std::endl;
    }

    {
        std::cout<<"Test 03: sf x df"<<std::endl;
        ros::Time time_before=ros::Time::now();
        for(int i=0; i<100; i++)
        {

            MatCdf=MatAsf*MatBdf;
        }
        ros::Time time_after=ros::Time::now();

        std::cout<<"\t delta time="<<(time_after-time_before)<<" ns"<<std::endl;
    }

    {
        std::cout<<"Test 04: df x sf"<<std::endl;
        ros::Time time_before=ros::Time::now();
        for(int i=0; i<100; i++)
        {

            MatCdf=MatAdf*MatBsf;
        }
        ros::Time time_after=ros::Time::now();

        std::cout<<"\t delta time="<<(time_after-time_before)<<" ns"<<std::endl;
    }


    {
        std::cout<<"Test 05: de x de"<<std::endl;
        ros::Time time_before=ros::Time::now();
        for(int i=0; i<100; i++)
        {

            MatCde=MatAde*MatBde;
        }
        ros::Time time_after=ros::Time::now();

        std::cout<<"\t delta time="<<(time_after-time_before)<<" ns"<<std::endl;
    }

    {
        std::cout<<"Test 06: se x se"<<std::endl;
        ros::Time time_before=ros::Time::now();
        for(int i=0; i<100; i++)
        {

            MatCse=MatAse*MatBse;
        }
        ros::Time time_after=ros::Time::now();

        std::cout<<"\t delta time="<<(time_after-time_before)<<" ns"<<std::endl;
    }

    {
        std::cout<<"Test 07: se x de"<<std::endl;
        ros::Time time_before=ros::Time::now();
        for(int i=0; i<100; i++)
        {

            MatCde=MatAse*MatBde;
        }
        ros::Time time_after=ros::Time::now();

        std::cout<<"\t delta time="<<(time_after-time_before)<<" ns"<<std::endl;
    }

    {
        std::cout<<"Test 08: de x se"<<std::endl;
        ros::Time time_before=ros::Time::now();
        for(int i=0; i<100; i++)
        {

            MatCde=MatAde*MatBse;
        }
        ros::Time time_after=ros::Time::now();

        std::cout<<"\t delta time="<<(time_after-time_before)<<" ns"<<std::endl;
    }




    {
        std::cout<<"Test 09: df x de"<<std::endl;
        ros::Time time_before=ros::Time::now();
        for(int i=0; i<100; i++)
        {

            MatCdf=MatAdf*MatBde;
        }
        ros::Time time_after=ros::Time::now();

        std::cout<<"\t delta time="<<(time_after-time_before)<<" ns"<<std::endl;
    }

    {
        std::cout<<"Test 10: sf x se"<<std::endl;
        ros::Time time_before=ros::Time::now();
        for(int i=0; i<100; i++)
        {

            MatCsf=MatAsf*MatBse;
        }
        ros::Time time_after=ros::Time::now();

        std::cout<<"\t delta time="<<(time_after-time_before)<<" ns"<<std::endl;
    }

    {
        std::cout<<"Test 11: sf x de"<<std::endl;
        ros::Time time_before=ros::Time::now();
        for(int i=0; i<100; i++)
        {

            MatCdf=MatAsf*MatBde;
        }
        ros::Time time_after=ros::Time::now();

        std::cout<<"\t delta time="<<(time_after-time_before)<<" ns"<<std::endl;
    }

    {
        std::cout<<"Test 12: df x se"<<std::endl;
        ros::Time time_before=ros::Time::now();
        for(int i=0; i<100; i++)
        {

            MatCdf=MatAdf*MatBse;
        }
        ros::Time time_after=ros::Time::now();

        std::cout<<"\t delta time="<<(time_after-time_before)<<" ns"<<std::endl;
    }






    {
        std::cout<<"Test 13: de x df"<<std::endl;
        ros::Time time_before=ros::Time::now();
        for(int i=0; i<100; i++)
        {

            MatCdf=MatAde*MatBdf;
        }
        ros::Time time_after=ros::Time::now();

        std::cout<<"\t delta time="<<(time_after-time_before)<<" ns"<<std::endl;
    }

    {
        std::cout<<"Test 14: se x sf"<<std::endl;
        ros::Time time_before=ros::Time::now();
        for(int i=0; i<100; i++)
        {

            MatCsf=MatAse*MatBsf;
        }
        ros::Time time_after=ros::Time::now();

        std::cout<<"\t delta time="<<(time_after-time_before)<<" ns"<<std::endl;
    }

    {
        std::cout<<"Test 15: se x df"<<std::endl;
        ros::Time time_before=ros::Time::now();
        for(int i=0; i<100; i++)
        {

            MatCdf=MatAse*MatBdf;
        }
        ros::Time time_after=ros::Time::now();

        std::cout<<"\t delta time="<<(time_after-time_before)<<" ns"<<std::endl;
    }

    {
        std::cout<<"Test 16: de x sf"<<std::endl;
        ros::Time time_before=ros::Time::now();
        for(int i=0; i<100; i++)
        {

            MatCdf=MatAde*MatBsf;
        }
        ros::Time time_after=ros::Time::now();

        std::cout<<"\t delta time="<<(time_after-time_before)<<" ns"<<std::endl;
    }




    {
        std::cout<<"Test 17: se x sf x se"<<std::endl;
        ros::Time time_before=ros::Time::now();
        for(int i=0; i<100; i++)
        {
            MatCsf=MatAse*MatBsf*MatAse;
        }
        ros::Time time_after=ros::Time::now();

        std::cout<<"\t delta time="<<(time_after-time_before)<<" ns"<<std::endl;
    }

    {
        std::cout<<"Test 18: de x df x de"<<std::endl;
        ros::Time time_before=ros::Time::now();
        for(int i=0; i<100; i++)
        {

            MatCdf=MatAde*MatBdf*MatAde;
        }
        ros::Time time_after=ros::Time::now();

        std::cout<<"\t delta time="<<(time_after-time_before)<<" ns"<<std::endl;
    }

    {
        std::cout<<"Test 19: se x df x se"<<std::endl;
        ros::Time time_before=ros::Time::now();
        for(int i=0; i<100; i++)
        {

            MatCdf=MatAse*MatBdf*MatAse;
        }
        ros::Time time_after=ros::Time::now();

        std::cout<<"\t delta time="<<(time_after-time_before)<<" ns"<<std::endl;
    }

    {
        std::cout<<"Test 20: de x sf x de"<<std::endl;
        ros::Time time_before=ros::Time::now();
        for(int i=0; i<100; i++)
        {

            MatCdf=MatAde*MatBsf*MatAde;
        }
        ros::Time time_after=ros::Time::now();

        std::cout<<"\t delta time="<<(time_after-time_before)<<" ns"<<std::endl;
    }


    return 0;
}
