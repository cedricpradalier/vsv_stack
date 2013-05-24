
#include<stdio.h>
#include <obstacle_mapper/MultiLevelCell.h>

using namespace obstacle_mapper;

int main() 
{
    MultiLevelCell mlc1;
    mlc1.update(0,1,MultiLevelCell::Free);
    mlc1.update(2,3,MultiLevelCell::Occupied);
    mlc1.update(4,5,MultiLevelCell::Free);
    mlc1.update(6,7,MultiLevelCell::Occupied);
    printf("MLC1: ");mlc1.print();printf("\n");
    assert(mlc1.validate());

    MultiLevelCell mlc2;
    mlc2.update(0,1,MultiLevelCell::Free);
    mlc2.update(1,2,MultiLevelCell::Occupied);
    mlc2.update(2,3,MultiLevelCell::Free);
    mlc2.update(3,4,MultiLevelCell::Occupied);
    printf("MLC2: ");mlc2.print();printf("\n");
    assert(mlc2.validate());

    MultiLevelCell mlc3;
    mlc3.update(0,1,MultiLevelCell::Free);
    mlc3.update(1,2,MultiLevelCell::Free);
    mlc3.update(3,4,MultiLevelCell::Free);
    mlc3.update(4,5,MultiLevelCell::Free);
    printf("MLC3: ");mlc3.print();printf("\n");
    assert(mlc3.validate());

    MultiLevelCell mlc4;
    mlc4.update(0,1,MultiLevelCell::Free);
    mlc4.update(2,3,MultiLevelCell::Occupied);
    mlc4.update(5,6,MultiLevelCell::Occupied);
    mlc4.update(0.5,2.5,MultiLevelCell::Free);
    printf("MLC4: ");mlc4.print();printf("\n");
    assert(mlc4.validate());

    MultiLevelCell mlc5;
    mlc5.update(0,1,MultiLevelCell::Free);
    mlc5.update(2,3,MultiLevelCell::Free);
    mlc5.update(0.5,2.5,MultiLevelCell::Free);
    printf("MLC5: ");mlc5.print();printf("\n");
    assert(mlc5.validate());

    MultiLevelCell mlc6;
    mlc6.update(0,1,MultiLevelCell::Free);
    mlc6.update(2,3,MultiLevelCell::Free);
    mlc6.update(0.5,2.5,MultiLevelCell::Occupied);
    printf("MLC6: ");mlc6.print();printf("\n");
    assert(mlc6.validate());


    return 0;
}

