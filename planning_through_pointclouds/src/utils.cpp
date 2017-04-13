//
// Utility functions
//
// David Butterworth
//

#include <planning_through_pointclouds/utils.h>

// c headers:
#include <stdlib.h>
#include <stdio.h>
#include <string.h> // strncmp
#include <sys/types.h>
#include <sys/sysinfo.h>

#include <iostream> // cout
#include <string>

#include <boost/program_options.hpp>
#include <boost/program_options/option.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options/value_semantic.hpp>

void initializeVectorOfBool(std::vector<std::vector<std::vector<std::vector<bool> > > >& vector,
                            const size_t dim_1_size, const size_t dim_2_size, const size_t dim_3_size,
                            const bool value)
{
  vector.resize(dim_1_size);

  for (size_t i = 0; i < dim_1_size; ++i)
  {
    vector.at(i).resize(dim_2_size);

    for (size_t j = 0; j < dim_2_size; ++j)
    {
      vector.at(i).at(j).resize(dim_3_size);

      for (size_t k = 0; k < dim_3_size; ++k)
      {
        vector.at(i).at(j).at(k).resize(1);

        for (size_t l = 0; l < 1; ++l)
        {
          vector.at(i).at(j).at(k).at(l) = value;
        }
      }
    }
  }
}

// Helper for boost::program_options
// Allows an argument to include a negative sign "--input -1.0"
// without being mistaken for a separate argument.
std::vector<boost::program_options::option> ignore_number_args(std::vector<std::string>& args)
{
  std::vector<boost::program_options::option> result;
  int pos = 0;
  while (!args.empty())
  {
    const auto& arg = args[0];
    double num;
    //if (boost::conversion::try_lexical_convert(arg, num)) // Requires newer Boost
    try
    {
      num = boost::lexical_cast<double>(arg);

      result.push_back(boost::program_options::option());
      boost::program_options::option& opt = result.back();

      opt.position_key = pos++;
      opt.value.push_back(arg);
      opt.original_tokens.push_back(arg);

      args.erase(args.begin());
    }
    //else
    catch (boost::bad_lexical_cast const&)
    {
      break;
    }
  }

  return result;
}

const bool processCommandLine(int argc, char** argv,
                              std::string& input_path,
                              std::vector<float>& start,
                              std::vector<float>& goal,
                              std::string& output_path)
{
  try
  {
    boost::program_options::options_description desc("Usage:");
    desc.add_options()
        ("help", "produce help message")
        ("input", boost::program_options::value<std::string>(&input_path)->required(), "/path/to/input/pointcloud.pcd")
        ("start", boost::program_options::value<std::vector<float> >()->multitoken()->required(), "(x,y,z)")
        ("goal", boost::program_options::value<std::vector<float> >()->multitoken()->required(), "(x,y,z)")
        ("output", boost::program_options::value<std::string>(&output_path)->required(), "/path/to/output_traj.ply")
    ;
    boost::program_options::variables_map vm;
    //boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::command_line_parser clparser(argc, argv);
    clparser.extra_style_parser(ignore_number_args);
    boost::program_options::store(clparser.options(desc).run(), vm);

    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        return 1;
    }

    // Warn if required arguments not specified
    boost::program_options::notify(vm);

    if (!vm["start"].empty() && (start = vm["start"].as<std::vector<float> >()).size() != 3)
    {
      std::cout << "ERROR: Start must be specified with 3 float values" << std::endl;
      return false;
    }

    if (!vm["goal"].empty() && (goal = vm["goal"].as<std::vector<float> >()).size() != 3)
    {
      // good to go
      std::cout << "ERROR: Goal must be specified with 3 float values" << std::endl;
      return false;
    }
  }
  catch(std::exception& e)
  {
    std::cerr << "ERROR: " << e.what() << "\n";
    return false;
  }
  catch(...)
  {
    std::cerr << "ERROR: Caught unknown exception" << "\n";
    return false;
  }

  return true;
}

const bool processCommandLine(int argc, char** argv,
                              std::string& input_path,
                              std::vector<float>& start,
                              std::vector<float>& goal,
                              std::string& planner,
                              double& max_planning_time,
                              std::string& output_path)
{
  try
  {
    boost::program_options::options_description desc("Usage:");
    desc.add_options()
        ("help", "produce help message")
        ("input", boost::program_options::value<std::string>(&input_path)->required(), "/path/to/input/pointcloud.pcd")
        ("start", boost::program_options::value<std::vector<float> >()->multitoken()->required(), "(x,y,z)")
        ("goal", boost::program_options::value<std::vector<float> >()->multitoken()->required(), "(x,y,z)")
        ("planner", boost::program_options::value<std::string>(&planner)->required(), "(RRT or RRT_CONNECT or RRT_STAR or BIT_STAR)")
        ("max_planning_time", boost::program_options::value<double>(&max_planning_time)->required(), "(seconds)")
        ("output", boost::program_options::value<std::string>(&output_path)->required(), "/path/to/output_traj.ply")
    ;
    boost::program_options::variables_map vm;
    //boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::command_line_parser clparser(argc, argv);
    clparser.extra_style_parser(ignore_number_args);
    boost::program_options::store(clparser.options(desc).run(), vm);

    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        return 1;
    }

    // Warn if required arguments not specified
    boost::program_options::notify(vm);

    if (!vm["start"].empty() && (start = vm["start"].as<std::vector<float> >()).size() != 3)
    {
      std::cout << "ERROR: Start must be specified with 3 float values" << std::endl;
      return false;
    }

    if (!vm["goal"].empty() && (goal = vm["goal"].as<std::vector<float> >()).size() != 3)
    {
      // good to go
      std::cout << "ERROR: Goal must be specified with 3 float values" << std::endl;
      return false;
    }
  }
  catch(std::exception& e)
  {
    std::cerr << "ERROR: " << e.what() << "\n";
    return false;
  }
  catch(...)
  {
    std::cerr << "ERROR: Caught unknown exception" << "\n";
    return false;
  }

  return true;
}

const bool processCommandLine3(int argc, char** argv,
                               std::string& input_path,
                               std::vector<float>& start,
                               std::vector<float>& goal,
                               double& min_height,
                               double& max_height,
                               std::string& planner,
                               double& max_planning_time,
                               std::string& output_path)
{
  try
  {
    boost::program_options::options_description desc("Usage:");
    desc.add_options()
        ("help", "produce help message")
        ("input", boost::program_options::value<std::string>(&input_path)->required(), "/path/to/input/pointcloud.pcd")
        ("start", boost::program_options::value<std::vector<float> >()->multitoken()->required(), "(x,y,z)")
        ("goal", boost::program_options::value<std::vector<float> >()->multitoken()->required(), "(x,y,z)")
        ("min_height", boost::program_options::value<double>(&min_height)->required(), "(z in meters)")
        ("max_height", boost::program_options::value<double>(&max_height)->required(), "(z in meters)")
        ("planner", boost::program_options::value<std::string>(&planner)->required(), "(RRT or RRT_CONNECT or RRT_STAR or BIT_STAR)")
        ("max_planning_time", boost::program_options::value<double>(&max_planning_time)->required(), "(seconds)")
        ("output", boost::program_options::value<std::string>(&output_path)->required(), "/path/to/output_traj.ply")
    ;
    boost::program_options::variables_map vm;
    //boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::command_line_parser clparser(argc, argv);
    clparser.extra_style_parser(ignore_number_args);
    boost::program_options::store(clparser.options(desc).run(), vm);

    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        return 1;
    }

    // Warn if required arguments not specified
    boost::program_options::notify(vm);

    if (!vm["start"].empty() && (start = vm["start"].as<std::vector<float> >()).size() != 3)
    {
      std::cout << "ERROR: Start must be specified with 3 float values" << std::endl;
      return false;
    }

    if (!vm["goal"].empty() && (goal = vm["goal"].as<std::vector<float> >()).size() != 3)
    {
      // good to go
      std::cout << "ERROR: Goal must be specified with 3 float values" << std::endl;
      return false;
    }
  }
  catch(std::exception& e)
  {
    std::cerr << "ERROR: " << e.what() << "\n";
    return false;
  }
  catch(...)
  {
    std::cerr << "ERROR: Caught unknown exception" << "\n";
    return false;
  }

  return true;
}

const bool processCommandLine(int argc, char** argv,
                              std::string& input_path,
                              std::vector<float>& start,
                              std::vector<float>& goal,
                              std::string& planner,
                              double& max_planning_time,
                              double& min_height,
                              double& turning_radius,
                              double& max_z_slope,
                              std::string& output_path)
{
  try
  {
    boost::program_options::options_description desc("Usage:");
    desc.add_options()
        ("help", "produce help message")
        ("input", boost::program_options::value<std::string>(&input_path)->required(), "/path/to/input/pointcloud.pcd")
        ("start", boost::program_options::value<std::vector<float> >()->multitoken()->required(), "(x,y,z)")
        ("goal", boost::program_options::value<std::vector<float> >()->multitoken()->required(), "(x,y,z)")
        ("planner", boost::program_options::value<std::string>(&planner)->required(), "(RRT or RRT_CONNECT or RRT_STAR or BIT_STAR)")
        ("max_planning_time", boost::program_options::value<double>(&max_planning_time)->required(), "(seconds)")
        ("min_height", boost::program_options::value<double>(&min_height)->required(), "(z in meters)")
        ("turning_radius", boost::program_options::value<double>(&turning_radius)->required(), "(rho)")
        ("max_z_slope", boost::program_options::value<double>(&max_z_slope)->required(), "(ratio)")
        ("output", boost::program_options::value<std::string>(&output_path)->required(), "/path/to/output_traj.ply")
    ;
    boost::program_options::variables_map vm;
    //boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::command_line_parser clparser(argc, argv);
    clparser.extra_style_parser(ignore_number_args);
    boost::program_options::store(clparser.options(desc).run(), vm);

    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        return 1;
    }

    // Warn if required arguments not specified
    boost::program_options::notify(vm);

    if (!vm["start"].empty() && (start = vm["start"].as<std::vector<float> >()).size() != 3)
    {
      std::cout << "ERROR: Start must be specified with 3 float values" << std::endl;
      return false;
    }

    if (!vm["goal"].empty() && (goal = vm["goal"].as<std::vector<float> >()).size() != 3)
    {
      // good to go
      std::cout << "ERROR: Goal must be specified with 3 float values" << std::endl;
      return false;
    }
  }
  catch(std::exception& e)
  {
    std::cerr << "ERROR: " << e.what() << "\n";
    return false;
  }
  catch(...)
  {
    std::cerr << "ERROR: Caught unknown exception" << "\n";
    return false;
  }

  return true;
}

const bool processCommandLine5(int argc, char** argv,
                               std::string& input_path,
                               std::vector<float>& start,
                               std::vector<float>& goal,
                               double& min_height,
                               double& max_height,
                               std::vector<float>& takeoff_funnel,
                               std::vector<float>& landing_funnel,
                               std::string& planner,
                               double& max_planning_time,
                               std::string& output_path)
{
  try
  {
    boost::program_options::options_description desc("Usage:");
    desc.add_options()
        ("help", "produce help message")
        ("input", boost::program_options::value<std::string>(&input_path)->required(), "/path/to/input/pointcloud.pcd")
        ("start", boost::program_options::value<std::vector<float> >(&start)->multitoken()->required(), "(x,y,z)")
        ("goal", boost::program_options::value<std::vector<float> >(&goal)->multitoken()->required(), "(x,y,z)")
        ("min_height", boost::program_options::value<double>(&min_height)->required(), "(z in meters)")
        ("max_height", boost::program_options::value<double>(&max_height)->required(), "(z in meters)")
        ("takeoff_funnel", boost::program_options::value<std::vector<float> >(&takeoff_funnel)->multitoken()->required(), "(offset, middle_radius, top_radius, cylinder_height, cone_height)")
        ("landing_funnel", boost::program_options::value<std::vector<float> >(&landing_funnel)->multitoken()->required(), "(offset, middle_radius, top_radius, cylinder_height, cone_height)")
        ("planner", boost::program_options::value<std::string>(&planner)->required(), "(RRT or RRT_CONNECT or RRT_STAR or BIT_STAR)")
        ("max_planning_time", boost::program_options::value<double>(&max_planning_time)->required(), "(seconds)")
        ("output", boost::program_options::value<std::string>(&output_path)->required(), "/path/to/output_traj.ply")
    ;
    boost::program_options::variables_map vm;
    //boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::command_line_parser clparser(argc, argv);
    clparser.extra_style_parser(ignore_number_args);
    boost::program_options::store(clparser.options(desc).run(), vm);

    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        return 1;
    }

    // Warn if required arguments not specified
    boost::program_options::notify(vm);

    if (!vm["start"].empty() && (start = vm["start"].as<std::vector<float> >()).size() != 3)
    {
      std::cout << "ERROR: Start must be specified with 3 float values" << std::endl;
      return false;
    }

    if (!vm["goal"].empty() && (goal = vm["goal"].as<std::vector<float> >()).size() != 3)
    {
      // good to go
      std::cout << "ERROR: Goal must be specified with 3 float values" << std::endl;
      return false;
    }

    if (!vm["takeoff_funnel"].empty() && (takeoff_funnel = vm["takeoff_funnel"].as<std::vector<float> >()).size() != 5)
    {
      // good to go
      std::cout << "ERROR: Argument 'takeoff_funnel' must be provided with 5 float values" << std::endl;
      return false;
    }

    if (!vm["landing_funnel"].empty() && (landing_funnel = vm["landing_funnel"].as<std::vector<float> >()).size() != 5)
    {
      // good to go
      std::cout << "ERROR: Argument 'landing_funnel' must be provided with 5 float values" << std::endl;
      return false;
    }
  }
  catch(std::exception& e)
  {
    std::cerr << "ERROR: " << e.what() << "\n";
    return false;
  }
  catch(...)
  {
    std::cerr << "ERROR: Caught unknown exception" << "\n";
    return false;
  }

  return true;
}







// Process command-line arguments.
// Expected syntax:
// executable --input /path/to/pointcloud.pcd --start 0.0 0.0 0.0 --goal 1.0 1.0 1.0 --min_height 5.0 --max_height 7.0 

//--planner RRT --max_planning_time 60.0 --output /path/to/output/traj.ply
/*
--takeoff_funnel 
offset, middle_radius, top_radius, cylinder_height, cone_height

--landing_funnel
*/

const bool processCommandLine6(int argc, char** argv,
                               std::string& path_to_pointcloud,
                               std::vector<float>& start,
                               std::vector<float>& goal,

                               double& robot_radius,

                               double& min_height,
                               double& max_height,
                               bool& use_height_constraints,

                               std::vector<float>& takeoff_funnel,
                               std::vector<float>& landing_funnel,
                               bool& use_funnel_constraints,

                               std::string& path_to_elevation_map,
                               double& min_altitude,
                               double& max_altitude,
                               bool& use_altitude_constraint,
                               bool& use_fixed_altitude_at_endpoints,

                               double& endpoints_z_offset,
                               bool& use_offset_endpoints_z_value,



                               std::string& planner,
                               double& max_planning_time,
                               std::string& output_path,
                               double& path_interpolation_step)
{
  //std::string fix_altitude_at_endpoints; // dummy variable

  try
  {
    boost::program_options::options_description desc("Usage:");
    desc.add_options()
        ("help", "produce help message")
        ("input", boost::program_options::value<std::string>(&path_to_pointcloud)->required(), "/path/to/input/pointcloud.pcd")
        
        ("start", boost::program_options::value<std::vector<float> >(&start)->multitoken()->required(), "(x,y,z)")
        ("goal", boost::program_options::value<std::vector<float> >(&goal)->multitoken()->required(), "(x,y,z)")

        ("robot_radius", boost::program_options::value<double>(&robot_radius), "(radius of the robot, in meters)")

        ("min_height", boost::program_options::value<double>(&min_height), "(z in meters)")
        ("max_height", boost::program_options::value<double>(&max_height), "(z in meters)")

        ("takeoff_funnel", boost::program_options::value<std::vector<float> >(&takeoff_funnel)->multitoken(), "(offset, middle_radius, top_radius, funnel_height)")
        ("landing_funnel", boost::program_options::value<std::vector<float> >(&landing_funnel)->multitoken(), "(offset, middle_radius, top_radius, funnel_height)")

        ("elevation_map", boost::program_options::value<std::string>(&path_to_elevation_map), "/path/to/elevation_map_pointcloud.pcd")
        ("min_altitude", boost::program_options::value<double>(&min_altitude), "(z in meters)")
        ("max_altitude", boost::program_options::value<double>(&max_altitude), "(z in meters)")

        ("fix_altitude_at_endpoints", boost::program_options::bool_switch()->default_value(false))

        ("endpoints_z_offset", boost::program_options::value<double>(&endpoints_z_offset), "(z distance in meters)")

        ("planner", boost::program_options::value<std::string>(&planner)->required(), "(RRT or RRT_CONNECT or RRT_STAR or BIT_STAR)")
        ("max_planning_time", boost::program_options::value<double>(&max_planning_time)->required(), "(seconds)")
        ("output", boost::program_options::value<std::string>(&output_path)->required(), "/path/to/output_traj.ply")

        ("path_interpolation_step", boost::program_options::value<double>(&path_interpolation_step), "(Distance between interpolated points, in meters)")
    ;
    boost::program_options::variables_map vm;
    //boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::command_line_parser clparser(argc, argv);
    clparser.extra_style_parser(ignore_number_args);
    boost::program_options::store(clparser.options(desc).run(), vm);

    if (vm.count("help"))
    {
        std::cout << desc << "\n";
        return 1;
    }

    // Warn if required arguments not specified
    boost::program_options::notify(vm);

    if (!vm["start"].empty() && (start = vm["start"].as<std::vector<float> >()).size() != 3)
    {
      std::cout << "ERROR: Start must be specified with 3 float values" << std::endl;
      return false;
    }

    if (!vm["goal"].empty() && (goal = vm["goal"].as<std::vector<float> >()).size() != 3)
    {
      // good to go
      std::cout << "ERROR: Goal must be specified with 3 float values" << std::endl;
      return false;
    }



    if (vm["robot_radius"].empty())
    {
      robot_radius = 0.0;
    }



    use_height_constraints = false;

    // One or both height constraints are optional
    if (vm["min_height"].empty())
    {
      // Default value when not specified
      min_height = std::numeric_limits<double>::min();
    }
    else
    {
      use_height_constraints = true;
    }

    if (vm["max_height"].empty())
    {
      // Default value when not specified
      max_height = std::numeric_limits<double>::max();
    }
    else
    {
      use_height_constraints = true;
    }

    std::cout << "use_height_constraints: " << use_height_constraints << std::endl;


    if (use_height_constraints && (!vm["takeoff_funnel"].empty() || !vm["landing_funnel"].empty()))
    {
      std::cout << "ERROR: You may specify height constraints OR funnel constraints, not both." << std::endl;
      return false;
    }




    if ((!vm["takeoff_funnel"].empty() && vm["landing_funnel"].empty())
        || (vm["takeoff_funnel"].empty() && !vm["landing_funnel"].empty()))
    {
      std::cout << "ERROR: To use funnel constraints, you must specify both 'takeoff_funnel' and 'landing_funnel'" << std::endl;
      return false;
    }



    if (!vm["takeoff_funnel"].empty() && (takeoff_funnel = vm["takeoff_funnel"].as<std::vector<float> >()).size() != 4)
    {
      std::cout << "ERROR: Argument 'takeoff_funnel' must be provided with 4 float values" << std::endl;
      return false;
    }
    if (!vm["landing_funnel"].empty() && (landing_funnel = vm["landing_funnel"].as<std::vector<float> >()).size() != 4)
    {
      std::cout << "ERROR: Argument 'landing_funnel' must be provided with 4 float values" << std::endl;
      return false;
    }


    if (vm["takeoff_funnel"].empty() && vm["landing_funnel"].empty())
    {
      use_funnel_constraints = false;
    }
    else
    {
      use_funnel_constraints = true;
    }






    unsigned int altitude_param_count = 0;
    if (vm["elevation_map"].empty())
    {
      use_altitude_constraint = false;
    }
    else
    {
      use_altitude_constraint = true;
      altitude_param_count++;
    }
    if (!vm["min_altitude"].empty())
    {
      altitude_param_count++;
    }
    if (!vm["max_altitude"].empty())
    {
      altitude_param_count++;
    }
    std::cout << "altitude_param_count: " << altitude_param_count << std::endl;
    if ((altitude_param_count != 0) && (altitude_param_count != 3))
    {
      std::cout << "ERROR: You must specify 'elevation_map' and 'min_altitude' and 'max_altitude'" << std::endl;
      return false;
    }

//std::cout << "vm["fix_altitude_at_endpoints"].as<bool>(): " << vm["fix_altitude_at_endpoints"].as<bool>() << std::endl;
    
    //if (vm["fix_altitude_at_endpoints"].empty())
    if (vm["fix_altitude_at_endpoints"].as<bool>() == false)
    {
      use_fixed_altitude_at_endpoints = false;
    }
    else
    {
      if (!use_altitude_constraint)
      {
        std::cout << "ERROR: You must specify 'elevation_map' and 'min_altitude' and 'max_altitude'" << std::endl;
        return false;
      }

      use_fixed_altitude_at_endpoints = true;
    }
    


    if (vm["endpoints_z_offset"].empty())
    {
      use_offset_endpoints_z_value = false;
      endpoints_z_offset = 0.0;
    }
    else
    {
      use_offset_endpoints_z_value = true;
    }



    if (vm["path_interpolation_step"].empty())
    {
      path_interpolation_step = 0.0;
    }






  }
  catch(std::exception& e)
  {
    std::cerr << "ERROR: " << e.what() << "\n";
    return false;
  }
  catch(...)
  {
    std::cerr << "ERROR: Caught unknown exception" << "\n";
    return false;
  }

  return true;
}





Timer::Timer()
{
  reset();
}
void Timer::reset()
{
  start_ = std::chrono::high_resolution_clock::now();
}
double Timer::read()
{
  auto end = std::chrono::high_resolution_clock::now();
  return std::chrono::duration<double, std::milli>(end - start_).count();
}

const uint parseLine(char* line)
{
  int i = strlen(line);
  const char* p = line;
  while (*p <'0' || *p > '9')
  {
    p++;
  }
  line[i-3] = '\0';
  i = atoi(p);
  return i;
}

const uint getProcessVirtualMemory()
{
  FILE* file = fopen("/proc/self/status", "r");
  int result = -1;
  char line[128];

  while (fgets(line, 128, file) != NULL)
  {
    if (strncmp(line, "VmSize:", 7) == 0)
    {
      result = parseLine(line);
      break;
    }
  }
  fclose(file);
  return result;
}

const uint getProcessPhysicalMemory()
{
  FILE* file = fopen("/proc/self/status", "r");
  int result = -1;
  char line[128];

  while (fgets(line, 128, file) != NULL)
  {
    if (strncmp(line, "VmRSS:", 6) == 0)
    {
      result = parseLine(line);
      break;
    }
  }
  fclose(file);
  return result;
}

const std::vector<std::string> splitString(const std::string& input, const char delimiter)
{
  std::istringstream iss(input);
  std::vector<std::string> tokens;
  std::string token;
  while (std::getline(iss, token, delimiter))
  {
    if (!token.empty())
    {
      tokens.push_back(token);
    }
  }
  return tokens;
}



const std::vector<double> range2(const double a, const double b, const double step_size, const bool include_endpoints)
{
  const double min_value =  std::min(a, b);
  const double max_value =  std::max(a, b);

  std::vector<double> values;
  if (include_endpoints)
  {
    values.push_back(min_value);
  }

  double p = min_value + step_size;
  while (1)
  {
    // If (b - a) < step_size then no other points will be added
    if (p > max_value)
    {
      break;
    }

    values.push_back(p);
    p += step_size;
  }

  if (std::fabs(values.back() - max_value) < 0.0001)
  {
    values.pop_back();
  }

  if (include_endpoints)
  {
    values.push_back(max_value);
  }

  if (a > b)
  {
    std::reverse(values.begin(), values.end());
    //return vec;
  }

  return values;
}

const std::vector<std::vector<double> > lerp(const std::vector<double>& q0, const std::vector<double>& q1, const double resolution)
{
  if (q0.size() != q1.size())
  {
    // throw
  }
  const int dims = q0.size();

  double sum = 0.0;
  for (int i = 0; i < dims; ++i)
  {
    sum += (q0.at(i)-q1.at(i))*(q0.at(i)-q1.at(i));
  }
  const double length = std::sqrt(sum);
  //std::cout << "len = " << length << std::endl;

  const bool include_endpoints = true;
  std::vector<double> t_values = range2(0.0, length, resolution, include_endpoints);
  //printStdVector<double>(t_values);

  // Normalize values between 0.0 to 1.0
  const double max_value = t_values.back();
  for (size_t i = 0; i < t_values.size(); ++i)
  {
    t_values.at(i) = t_values.at(i) / max_value;
  }
  t_values.back() = 1.0;
  //printStdVector<double>(t_values);

  std::vector<std::vector<double> > path;

  for (size_t i = 0; i < t_values.size(); ++i)
  {
    const double t = t_values.at(i);

    std::vector<double> point(dims);
    for (int j = 0; j < dims; ++j)
    {
      point.at(j) = q0.at(j) + t*(q1.at(j) - q0.at(j));
    }
    path.push_back(point);
  }

  return path;
}
