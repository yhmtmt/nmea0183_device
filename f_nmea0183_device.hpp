// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_nmea0183_device.hpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_nmea0183_device.hpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_nmea0183_device.hpp. If not, see <http://www.gnu.org/licenses/>. 

#ifndef F_NMEA0183_DEVICE_HPP
#define F_NMEA0183_DEVICE_HPP

#define SIZE_NMEA_BUF 166 // for two NMEA
#include "ch_vector.hpp"
#include "ch_nmea.hpp"

#include "filter_base.hpp"

class f_nmea0183_device: public f_base
{
protected:
  ch_nmea * m_chout;
  ch_nmea * m_chin;

  
  ch_nmea_data * m_data_out;
  c_nmea_dec m_decoder;
  
  bool m_blog;
  bool m_verb;
  char m_fname_log[1024];
  ofstream m_flog;
  char m_buf[SIZE_NMEA_BUF];
  char m_buf_send[SIZE_NMEA_BUF];
  char m_filter[6];

  int m_buf_head;
  int m_buf_tail;

  char m_nmea[84];
  int m_nmea_tail;
  char m_src_type_str[8];
  enum e_nmea_src{NONE, FILE, COM, UDP} m_nmea_src;
  union{
    char m_fname[1024];
    char m_host[1024];
  };

  // for serial port source
  unsigned short m_port;
  unsigned int m_cbr;

  AWS_SERIAL m_hcom;

  // for network source
  SOCKET m_sock;
  sockaddr_in m_sock_addr;
  char m_dst_host[256];
  SOCKET m_sock_out;
  sockaddr_in m_sock_addr_out;

  // for file source
  ifstream m_file;
  void dec_type_str(){
    if(strcmp("COM", m_src_type_str)==0){
      m_nmea_src = COM;
    }
    else if(strcmp("UDP", m_src_type_str) == 0){
      m_nmea_src = UDP;
    }
    else if(strcmp("FILE", m_src_type_str) == 0){
      m_nmea_src = FILE;
    }else{
      m_nmea_src = NONE;
    }
  }

  bool open_file();
  bool open_com();
  bool open_udp();

  bool rcv_file();
  bool rcv_com();
  bool rcv_udp();

  //extract_nmea_from_buffer() is the helper for rcv_com() and rcv_udp()
  void extract_nmea_from_buffer();

  int send_nmea();
  bool is_filtered(char * nmea){
    nmea++; // skip ! or $

    // comparing filter with five characters XXYYY where XX is the sender and YYY is the sentence code.
    for(int i = 0; i < 5; i++, nmea++){
      if(m_filter[i] == '*')
	continue;

      if(m_filter[i] != *nmea)
	return false;
    }
    return true;
  }

public:
  f_nmea0183_device(const char * name): f_base(name),
					m_chin(NULL), m_chout(NULL),
					m_data_out(nullptr),
					m_verb(false), m_blog(false),
			     m_hcom(NULL_SERIAL), m_nmea_src(NONE){
    m_fname[0] = '\0';

    m_filter[0] = m_filter[1] = m_filter[2] = m_filter[3] = m_filter[4] = '*';
    m_filter[5] = '\0';

    register_fpar("fnmea", m_fname, 1024, "File path of NMEA source file.");
    register_fpar("src_host", m_host, 1024, "IP address of NMEA source (if not specified ADDR_ANY is used).");
    register_fpar("dst_host", m_dst_host, 256, "IP address of NMEA destination (if not specified UDP output is not turned on."); 
    register_fpar("src", m_src_type_str, 128, "Source type. FILE or UDP or COM is allowed.");
    register_fpar("com", &m_port, "Number of NMEA source COM port.");
    register_fpar("bps", &m_cbr, "Baud rate of NMEA source COM port.");
    register_fpar("port", &m_port, "Port number of NMEA source UDP.");
    register_fpar("verb", &m_verb, "For debug.");
    register_fpar("log", &m_blog, "Log enable (y or n)");
    register_fpar("filter", m_filter, 6, "Sentence filter. 5 characters are to be specified. * can be used as wild card.");
    register_fpar("nmea_in", (ch_base**)&m_chin, typeid(ch_nmea).name(), "NMEA input channel.");
    register_fpar("nmea_out", (ch_base**)&m_chout, typeid(ch_nmea).name(), "NMEA output channel.");
    register_fpar("data_out", (ch_base**)&m_data_out, typeid(ch_nmea_data).name(), "Decoded data output channel.");
  }

  ~f_nmea0183_device(){
  }
	
  virtual bool seek(long long seek_time);
  virtual bool init_run();
  virtual bool proc();
  virtual void destroy_run();
};

#endif
