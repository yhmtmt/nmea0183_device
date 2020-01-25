// Copyright(c) 2012-2019 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// f_nmea0183_device.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_nmea0183_device.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_nmea0183_device.cpp.  If not, see <http://www.gnu.org/licenses/>. 
#include "f_nmea0183_device.hpp"
DEFINE_FILTER(f_nmea0183_device);

///////////////////////////////////////////////////////////// f_nmea0183_device
bool f_nmea0183_device::open_com()
{
  m_hcom = open_serial(m_fname, m_cbr);
  if(m_hcom == NULL_SERIAL){
    return false;
  }
  return true;

}

bool f_nmea0183_device::open_udp(){
  m_sock = socket(AF_INET, SOCK_DGRAM, 0);
  m_sock_addr.sin_family = AF_INET;
  m_sock_addr.sin_port = htons(m_port);

  set_sockaddr_addr(m_sock_addr, m_host);

  if(::bind(m_sock, (sockaddr*)&m_sock_addr, sizeof(m_sock_addr)) == SOCKET_ERROR){
    return false;
  }

  if(strlen(m_dst_host)){	
    m_sock_out = socket(AF_INET, SOCK_DGRAM, 0);
    m_sock_addr_out.sin_family = AF_INET;
    m_sock_addr_out.sin_port = htons(m_port);
    set_sockaddr_addr(m_sock_addr_out, m_dst_host);
    if(::bind(m_sock_out, (sockaddr*) & m_sock_addr_out, sizeof(m_sock_addr_out)) == SOCKET_ERROR){
      return false;
    }
  }
  return true;
}

bool f_nmea0183_device::seek(long long seek_time)
{
  if(m_nmea_src != FILE)
    return true;

  if(!m_file.is_open()){
    return false;
  }

  int prev = (int) m_file.tellg();
  tmex tm;
  while(!m_file.eof()){
    m_file.getline(m_buf, sizeof(m_buf));

    if(!decTmStr(m_buf, tm)){
      cerr << "f_nmea0183_device" << m_name << " cannot decode time in " << m_buf << endl;
      continue;
    }else{
      long long rec_time = mkgmtimeex_tz(tm, m_time_zone_minute) * MSEC;
      if(rec_time >= seek_time){
	m_file.seekg(prev);
	return true;
      }
    }
    prev = (int) m_file.tellg();
  }

  return false;
}

bool f_nmea0183_device::open_file(){
  m_file.open(m_fname);
  if(!m_file.is_open())
    return false;
  return true;
}


bool f_nmea0183_device::rcv_file()
{
  // file record is <time> <nmea>. <time> is 19 digit. <nmea> is 83 chars.

  if(!m_file.is_open()){
    return false;
  }

  long long rec_time = 0;
  do{
    if(m_file.eof())
      return false;

    int anchor = (int) m_file.tellg();

    m_file.getline(m_buf, sizeof(m_buf));
    tmex tm;
    if(!decTmStr(m_buf, tm)){
      cerr << "f_nmea0183_device " << m_name << " cannot decode time in " << m_buf << endl;
      continue;
    }else{
      rec_time = mkgmtimeex_tz(tm, m_time_zone_minute) * MSEC;
    }

    if(rec_time > get_time()){
      m_file.seekg(anchor);
      break;
    }

    //cout << &m_buf[31] << endl;
    if(m_chout){
      int len = (int) strlen(&m_buf[31]);
      if(len >= 84){
	cerr << "Irregal long sentence detected. :" << m_buf << endl;
	cerr << "the length is " << len << endl;
	continue;
      }
      if(is_filtered(&m_buf[31])){
	if(!m_chout->push(&m_buf[31]))
	  cerr << "Buffer overflow in ch_nmea " << m_chout->get_name() << endl;
      }
    }

    if(m_blog){
      m_flog << get_time_str() << &m_buf[31] << endl;
    }

  }while(rec_time <= get_time());
  return true;
}

bool f_nmea0183_device::rcv_com()
{
  int nrcv;
  while(nrcv = read_serial(m_hcom, &m_buf[m_buf_head], SIZE_NMEA_BUF - m_buf_head)){
    m_buf_tail += nrcv;
    extract_nmea_from_buffer();
  }
  return true;
}

bool f_nmea0183_device::rcv_udp()
{
  int nrcv;
  while(nrcv = recv(m_sock, m_buf + m_buf_head, SIZE_NMEA_BUF - m_buf_head, 0)){
    m_buf_tail += nrcv;
    extract_nmea_from_buffer();
  }
  return true;
}

void f_nmea0183_device::extract_nmea_from_buffer()
{
  while(m_buf_tail){
    // if the nmea sentence is empty, seek for the head mark ! or $.
    if(m_nmea_tail == 0){
      for(;m_buf[m_buf_head] != '!' 
	    && m_buf[m_buf_head] != '$' 
	    && m_buf_head < m_buf_tail; m_buf_head++){
	if(!m_buf[m_buf_head]){
	  m_buf_head = m_buf_tail = m_nmea_tail = 0;
	  return;
	}
      }

      if(m_buf_head == m_buf_tail){
	m_buf_head = 0;
	m_buf_tail = 0;
	break;
      }
    }

    // Copy the nmea string from buffer 
    for(;m_buf_head < m_buf_tail && m_nmea_tail < 84; 
	m_buf_head++, m_nmea_tail++){
      if(!m_buf[m_buf_head]){
	m_buf_head = m_buf_tail = m_nmea_tail = 0;
	return;
      }

      m_nmea[m_nmea_tail] = m_buf[m_buf_head];
      if(m_buf[m_buf_head] == 0x0D){// detecting CR
	m_nmea[m_nmea_tail] = '\0';
	m_nmea_tail = -1; // uses -1 as flag of nmea extraction
	break;
      }else if(m_nmea_tail == 84){
	m_nmea[83] = '\0';
	cerr << m_name << "::extrac_nmea_from_buffer" << 
	  " No termination character detected in 83 characters." << endl;
	cerr << "    -> string: " << m_nmea << endl;
	m_nmea_tail = 0;
      }
    }

    // if the  buffer ends before the full nmea sentence is transfered
    // we need to fill buffer with newly received data.
    if(m_buf_head == m_buf_tail){
      m_buf_head = 0;
      m_buf_tail = 0;
      break;
    }

    // If the complete nmea found, send it to the channel and log file.
    // Then the string remained in the buffer is shiftted to the head.
    if(m_nmea_tail == -1){
      if(is_filtered(m_nmea)){
	if(!m_chout->push(m_nmea)){
	  cerr << "Buffer overflow in ch_nmea " << m_chout->get_name() << endl;
	}
      }

      if(m_blog){
	if(m_flog.is_open())
	  m_flog << get_time_str() << m_nmea << endl;
	else{
	  sprintf(m_fname_log, "%s_%lld.nmea", m_name, get_time());
	  m_flog.open(m_fname_log);
	}
      }

      if(m_verb){
	cout << m_name << " > " << m_nmea << endl;
      }

      // remained data moves to the buffer head
      int itail = 0;
      for(; m_buf_head < m_buf_tail; m_buf_head++, itail++){
	m_buf[itail] = m_buf[m_buf_head];
      }
      m_buf_head = 0; 
      m_buf_tail = itail;
      m_nmea_tail = 0;
    }
  }
}

bool f_nmea0183_device::proc(){
  int total = 0;
  int nrcv = 0;
  while(send_nmea());

  switch(m_nmea_src){
  case FILE:
    if(!rcv_file())
      return false;
    break;
  case COM:
    if(!rcv_com())
      return false;
    break;
  case UDP:
    if(!rcv_udp())
      return false;
    break;
  }
  return true;
}

int f_nmea0183_device::send_nmea()
{
  if(!m_chin){
    return 0;
  }

  int len = 0;
  if(m_chin->pop(m_buf_send)){
    int _len = (int) strlen(m_buf_send);
    if(m_buf_send[_len-2] != 0x0D && m_buf_send[_len-1] != 0x0A){
      m_buf_send[_len] = 0x0D;
      m_buf_send[_len+1] = 0x0A;
      m_buf_send[_len+2] = 0x00;
    }

    if(m_blog){
      m_flog << get_time_str() << m_buf_send << endl;
    }
    if(m_verb){
      cout << m_name << " > " << m_buf_send << endl;
    }
		
    switch(m_nmea_src){
    case FILE:
      cout << m_buf_send << endl;
      len = (int) strlen(m_buf_send);
      break;
    case COM:
      len = write_serial(m_hcom, m_buf_send, (int) strlen(m_buf_send));
      break;
    case UDP:
      len = send(m_sock_out, m_buf_send, (int) sizeof(m_buf_send), 0);
      break;
    }
  }
  return len;
}

bool f_nmea0183_device::init_run()
{

  for(int ich = 0;ich < f_base::m_chout.size(); ich++){
    ch_nmea * pch = dynamic_cast<ch_nmea*>(f_base::m_chout[ich]);
    if(pch != NULL){
      m_chout = pch;
    }
  }

  for(int ich = 0; ich < f_base::m_chin.size(); ich++){
    ch_nmea * pch = dynamic_cast<ch_nmea*>(f_base::m_chin[ich]);
    if(pch != NULL)
      m_chin = pch;
  }

  dec_type_str();
  switch(m_nmea_src){
  case NONE:
    return false;
  case FILE:
    if(!open_file())
      return false;
    break;
  case COM:
    if(!open_com())
      return false;
    break;
  case UDP:
    if(!open_udp())
      return false;
    break;
  }
  if(m_blog){
    time_t t = time(NULL);
    sprintf(m_fname_log, "%s_%lld.nmea", m_name, (long long int)t);
    m_flog.open(m_fname_log);
    if(!m_flog.is_open()){
      destroy_run();
      return false;
    }
  }

  m_buf_head = 0;
  m_buf_tail = 0;
  m_nmea_tail = 0;
  return true;
}

void f_nmea0183_device::destroy_run(){
  switch(m_nmea_src){
  case FILE:
    m_file.close();
    break;
  case COM:
    ::close(m_hcom);
    m_hcom = 0;
    break;
  case UDP:
    ::close(m_sock);
    break;
  }
  
  if(m_flog.is_open())
    m_flog.close();
}
