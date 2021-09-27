#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

#include"clipper_path_io.h"

using namespace std;
using namespace ClipperLib;

//---------------------------------------------------------------------------
// SVGBuilder class
// a very simple class that creates an SVG image file
//---------------------------------------------------------------------------
  static string ColorToHtml(unsigned clr)
  {
    stringstream ss;
    ss << '#' << hex << std::setfill('0') << setw(6) << (clr & 0xFFFFFF);
    return ss.str();
  }
  static float GetAlphaAsFrac(unsigned clr)
  {
    return ((float)(clr >> 24) / 255);
  }

  bool ASCII_icompare(const char* str1, const char* str2)
  {
      //case insensitive compare for ASCII chars only
      while (*str1)
      {
          if (toupper(*str1) != toupper(*str2)) return false;
          str1++;
          str2++;
      }
      return (!*str2);
  }
  void MakeRandomPoly(int edgeCount, int width, int height, Paths& poly)
  {
      poly.resize(1);
      poly[0].resize(edgeCount);
      for (int i = 0; i < edgeCount; i++) {
          poly[0][i].X = rand() % width;
          poly[0][i].Y = rand() % height;
      }
  }
const std::string SVGBuilder::svg_xml_start [] =
  {"<?xml version=\"1.0\" standalone=\"no\"?>\n"
    "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.0//EN\"\n"
    "\"http://www.w3.org/TR/2001/REC-SVG-20010904/DTD/svg10.dtd\">\n\n"
    "<svg width=\"",
    "\" height=\"",
    "\" viewBox=\"0 0 ",
    "\" version=\"1.0\" xmlns=\"http://www.w3.org/2000/svg\">\n\n"
  };
const std::string SVGBuilder::poly_end [] =
  {"\"\n style=\"fill:",
    "; fill-opacity:",
    "; fill-rule:",
    "; stroke:",
    "; stroke-opacity:",
    "; stroke-width:",
    ";\"/>\n\n"
  };  
//------------------------------------------------------------------------------
  void SVGBuilder::AddPaths(Paths& poly)
  {
    if (poly.size() == 0) return;
    polyInfos.push_back(PolyInfo(poly, style));
  }

bool SVGBuilder::SaveToFile(const string& filename, double scale , int margin )
  {
    //calculate the bounding rect ...
    PolyInfoList::size_type i = 0;
    Paths::size_type j;
    while (i < polyInfos.size())
    {
      j = 0;
      while (j < polyInfos[i].paths.size() &&
        polyInfos[i].paths[j].size() == 0) j++;
      if (j < polyInfos[i].paths.size()) break;
      i++;
    }
    if (i == polyInfos.size()) return false;

    IntRect rec;
    rec.left = polyInfos[i].paths[j][0].X;
    rec.right = rec.left;
    rec.top = polyInfos[i].paths[j][0].Y;
    rec.bottom = rec.top;
    for ( ; i < polyInfos.size(); ++i)
      for (Paths::size_type j = 0; j < polyInfos[i].paths.size(); ++j)
        for (Path::size_type k = 0; k < polyInfos[i].paths[j].size(); ++k)
        {
          IntPoint ip = polyInfos[i].paths[j][k];
          if (ip.X < rec.left) rec.left = ip.X;
          else if (ip.X > rec.right) rec.right = ip.X;
          if (ip.Y < rec.top) rec.top = ip.Y;
          else if (ip.Y > rec.bottom) rec.bottom = ip.Y;
        }

    if (scale == 0) scale = 1.0;
    if (margin < 0) margin = 0;
    rec.left = (cInt)((double)rec.left * scale);
    rec.top = (cInt)((double)rec.top * scale);
    rec.right = (cInt)((double)rec.right * scale);
    rec.bottom = (cInt)((double)rec.bottom * scale);
    cInt offsetX = -rec.left + margin;
    cInt offsetY = -rec.top + margin;

    ofstream file;
    file.open(filename);
    if (!file.is_open()) return false;
    file.setf(ios::fixed);
    file.precision(0);
    file << svg_xml_start[0] <<
      ((rec.right - rec.left) + margin*2) << "px" << svg_xml_start[1] <<
      ((rec.bottom - rec.top) + margin*2) << "px" << svg_xml_start[2] <<
      ((rec.right - rec.left) + margin*2) << " " <<
      ((rec.bottom - rec.top) + margin*2) << svg_xml_start[3];
    setlocale(LC_NUMERIC, "C");
    file.precision(2);

    for (PolyInfoList::size_type i = 0; i < polyInfos.size(); ++i)
  {
      file << " <path d=\"";
    for (Paths::size_type j = 0; j < polyInfos[i].paths.size(); ++j)
      {
        if (polyInfos[i].paths[j].size() < 3) continue;
        file << " M " << ((double)polyInfos[i].paths[j][0].X * scale + offsetX) <<
          " " << ((double)polyInfos[i].paths[j][0].Y * scale + offsetY);
        for (Path::size_type k = 1; k < polyInfos[i].paths[j].size(); ++k)
        {
          IntPoint ip = polyInfos[i].paths[j][k];
          double x = (double)ip.X * scale;
          double y = (double)ip.Y * scale;
          file << " L " << (x + offsetX) << " " << (y + offsetY);
        }
        file << " z";
    }
      file << poly_end[0] << ColorToHtml(polyInfos[i].si.brushClr) <<
    poly_end[1] << GetAlphaAsFrac(polyInfos[i].si.brushClr) <<
        poly_end[2] <<
        (polyInfos[i].si.pft == pftEvenOdd ? "evenodd" : "nonzero") <<
        poly_end[3] << ColorToHtml(polyInfos[i].si.penClr) <<
    poly_end[4] << GetAlphaAsFrac(polyInfos[i].si.penClr) <<
        poly_end[5] << polyInfos[i].si.penWidth << poly_end[6];

        if (polyInfos[i].si.showCoords)
        {
      file << "<g font-family=\"Verdana\" font-size=\"11\" fill=\"black\">\n\n";
      for (Paths::size_type j = 0; j < polyInfos[i].paths.size(); ++j)
      {
        if (polyInfos[i].paths[j].size() < 3) continue;
        for (Path::size_type k = 0; k < polyInfos[i].paths[j].size(); ++k)
        {
          IntPoint ip = polyInfos[i].paths[j][k];
          file << "<text x=\"" << (int)(ip.X * scale + offsetX) <<
          "\" y=\"" << (int)(ip.Y * scale + offsetY) << "\">" <<
          ip.X << "," << ip.Y << "</text>\n";
          file << "\n";
        }
      }
      file << "</g>\n";
        }
    }
    file << "</svg>\n";
    file.close();
    setlocale(LC_NUMERIC, "");
    return true;
  }

bool SVGBuilder::SaveToBinaryFile(const string& filename, Paths &ppg, double scale , unsigned decimal_places )
{
  ofstream ofs(filename);
  if (!ofs) return false;

  if (decimal_places > 8) decimal_places = 8;
  ofs << setprecision(decimal_places) << std::fixed;

  Path pg;
  for (size_t i = 0; i < ppg.size(); ++i)
  {
    for (size_t j = 0; j < ppg[i].size(); ++j)
      ofs << ppg[i][j].X / scale << ", " << ppg[i][j].Y / scale << "," << std::endl;
    ofs << std::endl;
  }
  ofs.close();
  return true;
}
//------------------------------------------------------------------------------

bool SVGBuilder::LoadFromBinaryFile(Paths &ppg, const string& filename, double scale)
{
  //file format assumes: 
  //  1. path coordinates (x,y) are comma separated (+/- spaces) and 
  //  each coordinate is on a separate line
  //  2. each path is separated by one or more blank lines

  ppg.clear();
  ifstream ifs(filename);
  if (!ifs) return false;
  string line;
  Path pg;
  while (std::getline(ifs, line))
  {
    stringstream ss(line);
    double X = 0.0, Y = 0.0;
    if (!(ss >> X))
    {
      //ie blank lines => flag start of next polygon 
      if (pg.size() > 0) ppg.push_back(pg);
      pg.clear();
      continue;
    }
    char c = ss.peek();  
    while (c == ' ') {ss.read(&c, 1); c = ss.peek();} //gobble spaces before comma
    if (c == ',') {ss.read(&c, 1); c = ss.peek();} //gobble comma
    while (c == ' ') {ss.read(&c, 1); c = ss.peek();} //gobble spaces after comma
    if (!(ss >> Y)) break; //oops!
    pg.push_back(IntPoint((cInt)(X * scale),(cInt)(Y * scale)));
  }
  if (pg.size() > 0) ppg.push_back(pg);
  ifs.close();
  return true;
}
