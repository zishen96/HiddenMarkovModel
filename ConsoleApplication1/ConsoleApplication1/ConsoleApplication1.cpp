#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <iomanip>

//Grid base
const int row = 6;
const int col = 7;
std::string grid[row][col];
float priorGrid[row][col] = { 0 };  //Keeps track of updating prior probabilities
float evGrid[row][col] = { 0 };     //Keep track of updating evidence probabilities   
float evNormTerm = 0.0;

void initGrid()
{
    //11 obstacles, 31 free spaces
    grid[1][2] = "####";
    grid[1][3] = "####";
    grid[1][4] = "####";
    grid[1][5] = "####";
    grid[2][2] = "####";
    grid[2][5] = "####";
    grid[3][1] = "####";
    grid[3][2] = "####";
    grid[3][5] = "####";
    grid[4][4] = "####";
    grid[4][5] = "####";

    //31 free spaces it can start on, show percentage
    float prob = (float)1.0 / 31;

    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            if (grid[i][j] != "####")
            {
                priorGrid[i][j] = prob;
                grid[i][j] = std::to_string(prob*100.0);
            }
        }
    }
}

float roundTwoDec(float var)
{
    float val = (int)(var * 100 + 0.5);
    return (float)val / 100;
}

void draw()
{
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            if (grid[i][j] != "####")
            {
                std::cout << roundTwoDec(std::stof(grid[i][j])) << "\t";
            }
            else
                std::cout << grid[i][j] << "\t";
        }
        std::cout << "\n\n";
    }
    std::cout << "****************************************************\n";
}

void evidenceProb(std::string w, std::string n, std::string e, std::string s)
{
    //Calculate P(Z=(w,n,e,s)|S)
    float correctSpace = 0.9;       //"-" sensed as "-" = 90 %
    float incorrectSpace = 0.1;     //"-" sensed as "O" = 10 %
    float correctObstacle = 0.85;   //"O" sensed as "O" = 85 %
    float incorrectObstacle = 0.15; //"O" sensed as "-" = 15 %
    bool westObstacle = false, northObstacle = false, eastObstacle = false, southObstacle = false;
    float west = 0.0, north = 0.0, east = 0.0, south = 0.0, evidenceProbability = 0.0;

    evNormTerm = 0.0;
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            //For each grid space, reset flags
            westObstacle = false;
            northObstacle = false;
            eastObstacle = false;
            southObstacle = false;

            if (grid[i][j] != "####")
            {
                //The accurate obstacle flags are set here
                if (j - 1 < 0 || grid[i][j - 1] == "####")
                {
                    westObstacle = true;
                }
                if (i - 1 < 0 || grid[i - 1][j] == "####")
                {
                    northObstacle = true;
                }
                if (j + 1 >= col || grid[i][j + 1] == "####")
                {
                    eastObstacle = true;
                }
                if (i + 1 >= row || grid[i + 1][j] == "####")
                {
                    southObstacle = true;
                }

                //Compare set flags to what was passed in as sensor inputs
                switch (westObstacle)
                {
                case false:
                    if (w == "-") { west = correctSpace; }
                    else if (w == "o") { west = incorrectSpace; }
                    break;
                case true:
                    if (w == "o") { west = correctObstacle; }
                    else if (w == "-") { west = incorrectObstacle; }
                    break;
                }

                switch (northObstacle)
                {
                case false:
                    if (n == "-") { north = correctSpace; }
                    else if (n == "o") { north = incorrectSpace; }
                    break;
                case true:
                    if (n == "o") { north = correctObstacle; }
                    else if (n == "-") { north = incorrectObstacle; }
                    break;
                }

                switch (eastObstacle)
                {
                case false:
                    if (e == "-") { east = correctSpace; }
                    else if (e == "o") { east = incorrectSpace; }
                    break;
                case true:
                    if (e == "o") { east = correctObstacle; }
                    else if (e == "-") { east = incorrectObstacle; }
                    break;
                }

                switch (southObstacle)
                {
                case false:
                    if (s == "-") { south = correctSpace; }
                    else if (s == "o") { south = incorrectSpace; }
                    break;
                case true:
                    if (s == "o") { south = correctObstacle; }
                    else if (s == "-") { south = incorrectObstacle; }
                    break;
                }

                evidenceProbability = west * north * east * south;
                evGrid[i][j] = evidenceProbability;

                //Sum the normalization term for filter calculations
                evNormTerm += evGrid[i][j] * priorGrid[i][j];
            }
        }
    }
}

float transition(std::string dir, int i, int j)
{
    /*
    Given desired direction and specific grid space, calculate the total prob
    Moving towards desired direction - 80%
    Drifting left or right - 10% each
    */
    float driftProb = 0.1;
    float desiredProb = 0.8;
    float totalProb = 0.0;

    //Given desired direction, account for whichever other spaces lead into each grid space
    if (dir == "w")
    {
        /*Desired direction is West
        80% movement from East - desiredProb
        80% movement from bouncing back - desiredProb
        10% movement from North - driftProb
        10% movement from South - driftProb
        */
        
        //If movement coming from east grid is possible, take that into account. Otherwise its 0%
        if (j + 1 < col && grid[i][j + 1] != "####")
        {
            totalProb += desiredProb * priorGrid[i][j + 1];
        }
        //If moving west bounces into the wall or an obstacle
        if (j - 1 < 0 || grid[i][j - 1] == "####")
        {
            totalProb += desiredProb * priorGrid[i][j];
        }
        //If drifting north bounces into the wall or an obstacle
        if (i - 1 < 0 || grid[i - 1][j] == "####")
        {
            totalProb += driftProb * priorGrid[i][j];
        }
        else
        {
            totalProb += driftProb * priorGrid[i-1][j];
        }
        //If drifting south bounces into the wall or an obstacle
        if (i + 1 >= row || grid[i + 1][j] == "####")
        {
            totalProb += driftProb * priorGrid[i][j];
        }
        else
        {
            totalProb += driftProb * priorGrid[i+1][j];
        }
    }
    else if (dir == "n")
    {
        /*Desired direction is North
        80% movement from South - desiredProb
        80% movement from bouncing back - desiredProb
        10% movement from West - driftProb
        10% movement from East - driftProb
        */
        
        //If movement coming from south grid space is possible
        if (i + 1 < row && grid[i + 1][j] != "####")
        {
            totalProb += desiredProb * priorGrid[i + 1][j];
        }
        //If moving north bounces into the wall or an obstacle
        if (i - 1 < 0 || grid[i - 1][j] == "####")
        {
            totalProb += desiredProb * priorGrid[i][j];
        }
        //If drifting left bounces into the wall or an obstacle
        if (j - 1 < 0 || grid[i][j - 1] == "####")
        {
            totalProb += driftProb * priorGrid[i][j];
        }
        else
        {
            totalProb += driftProb * priorGrid[i][j - 1];
        }
        //If drifting right bounces into the wall or an obstacle
        if (j + 1 >= col || grid[i][j + 1] == "####")
        {
            totalProb += driftProb * priorGrid[i][j];
        }
        else
        {
            totalProb += driftProb * priorGrid[i][j + 1];
        }
    }
    else if (dir == "e")
    {
        /*Desired direction is East
        80% movement from West - desiredProb
        80% movement from bouncing back - desiredProb
        10% movement from North - driftProb
        10% movement from South - driftProb
        */

        //If movement coming from west grid space is possible
        if (j - 1 >= 0 && grid[i][j - 1] != "####")
        {
            totalProb += desiredProb * priorGrid[i][j - 1];
        }
        //If moving east bounces into the wall/obstacle
        if (j + 1 >= col || grid[i][j + 1] == "####")
        {
            totalProb += desiredProb * priorGrid[i][j];
        }
        //If drifting north bounces into the wall/obstacle
        if (i - 1 < 0 || grid[i - 1][j] == "####")
        {
            totalProb += driftProb * priorGrid[i][j];
        }
        else
        {
            totalProb += driftProb * priorGrid[i-1][j];
        }
        //If drifting south bounces into the wall/obstacle
        if (i + 1 >= row || grid[i + 1][j] == "####")
        {
            totalProb += driftProb * priorGrid[i][j];
        }
        else
        {
            totalProb += driftProb * priorGrid[i+1][j];
        }
    }
    else if (dir == "s")
    {
        /*Desired direction is South
        80% movement from North - desiredProb
        80% movement from bouncing back - desiredProb
        10% movement from West - driftProb
        10% movement from East - driftProb
        */

        //If movement coming from north grid space is possible
        if (i-1 >= 0 && grid[i-1][j] != "####")
        {
            totalProb += desiredProb * priorGrid[i - 1][j];
        }
        //If moving south bounces into the wall or an obstacle
        if (i+1 >= row || grid[i+1][j] == "####")
        {
            totalProb += desiredProb * priorGrid[i][j];
        }
        //If drifting left bounces into the wall or an obstacle
        if (j - 1 < 0 || grid[i][j - 1] == "####")
        {
            totalProb += driftProb * priorGrid[i][j];
        }
        else
        {
            totalProb += driftProb * priorGrid[i][j - 1];
        }
        //If drifting right bounces into the wall or an obstacle
        if (j + 1 >= col || grid[i][j + 1] == "####")
        {
            totalProb += driftProb * priorGrid[i][j];
        }
        else
        {
            totalProb += driftProb * priorGrid[i][j + 1];
        }
    }
    return totalProb;
}

void updatePrior()
{
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            if (grid[i][j] != "####")
            {
                priorGrid[i][j] = std::stof(grid[i][j]);
            }
        }
    }
}

void filter(std::string w, std::string n, std::string e, std::string s)
{
    //Calculate P(Z|S)P(S)
    //Evidence probability * prior / normalization term = individual position %
    
    evidenceProb(w, n, e, s);
    //For each grid space, calculate its normalized probability
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            if (grid[i][j] != "####")
            {
                //Grid displays %, so *100 for value
                grid[i][j] = std::to_string(evGrid[i][j]*priorGrid[i][j]/evNormTerm*100);
            }
        }
    }
    updatePrior();
}

void predictAfterAction(std::string dir)
{
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            if (grid[i][j] != "####")
            {
                grid[i][j] = std::to_string(transition(dir, i, j));
            }
        }
    }

    updatePrior();
}


int main()
{
    std::cout << "Initial location probabilities\n";
    initGrid();
    draw();

    std::cout << "Filtering after evidence [-, -, -, o]\n";
    filter("-", "-", "-", "o");
    draw();

    std::cout << "Prediction after Action N\n";
    predictAfterAction("n");
    draw();

    std::cout << "Filtering after evidence [-, o, -, -]\n";
    filter("-", "o", "-", "-");
    draw();

    std::cout << "Prediction after Action E\n";
    predictAfterAction("e");
    draw();

    std::cout << "Filtering after evidence [-, o, -, -]\n";
    filter("-", "o", "-", "-");
    draw();

    std::cout << "Prediction after Action E\n";
    predictAfterAction("e");
    draw();

    std::cout << "Filtering after evidence [-, -, o, -]\n";
    filter("-", "-", "o", "-");
    draw();
}