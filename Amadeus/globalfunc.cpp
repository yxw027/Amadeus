//#include "stdafx.h"
//#include "globalfunc.h"
//#include "EBAS_V_UI_R.h"
//
//extern EBAS_V_UI_RApp theApp;
//
//
//static void clearnetlist()
//{
//	for (NetList::iterator i = theApp.m_NetList.begin(); i != theApp.m_NetList.end(); i++)
//	{
//		for (SiteList::iterator j = i->second->m_SiteList.begin(); j != i->second->m_SiteList.end(); j++)
//		{
//			if (j->second->psock.state)
//			{
//				j->second->close();
//			}
//			delete j->second;
//		}
//		i->second->m_SiteList.clear();
//	}
//	theApp.m_NetList.clear();
//}
//
//void UpdateNetList(void)
//{
//	clearnetlist();
//	NetList NetTemp;
//	CString cstr;
//	// 更新列表
//	try
//	{
//		CStringArray   cstrList;
//		// 获取子网名称列表
//		m_sql.getnetlist(cstrList);
//		int size = cstrList.GetCount();
//		for (int i = 0; i < size; i++)
//		{
//			// 获取子网名称
//			CString NetName = cstrList.GetAt(i);
//			NetStr* pNet = new NetStr();
//			pNet->m_runflag = false;
//			NetTemp[NetName] = pNet;
//			m_sql.update_netinfo2str(NetName, &pNet->m_Netinfo);
//			if (theApp.m_NetList.find(NetName) != theApp.m_NetList.end())
//			{
//				pNet->m_Server = theApp.m_NetList.find(NetName)->second->m_Server;
//			}
//			else
//			{
//				// start server
//			}
//			// 挂载点列表筛选
//			CStringArray MountPointList;
//			m_sql.getsitelist(NetName, MountPointList);
//			int MPsize = MountPointList.GetCount();
//			for (int site_i = 0; site_i < MPsize; site_i++)
//			{
//				// 获取  ID 号
//				cstr = MountPointList.GetAt(site_i);
//				CStringArray cstrinfo;
//				m_sql.getsiteinfo(cstr, cstrinfo);
//				if (cstrinfo.GetCount())
//					cstr = cstrinfo.GetAt(0);
//				else
//					continue;
//				if (theApp.m_NetList.find(NetName) != theApp.m_NetList.end()
//					&& theApp.m_NetList.find(NetName)->second->m_SiteList.find(cstr)
//					!= theApp.m_NetList.find(NetName)->second->m_SiteList.end())
//				{
//					NetTemp[NetName]->m_SiteList[cstr] = theApp.m_NetList.find(NetName)->second->m_SiteList.find(cstr)->second;
//					continue;
//				}
//				/*"ID","MOUNTPOINT","COM_TYPE","IP","PORT","USERNAME","PASSWORD","LATITUDE","LONGITUDE","HEIGHT"*/
//				int index = 1;
//				CNtripClient*  temp = new CNtripClient;
//
//				sprintf(temp->rtcm_fwrt.workpath, pNet->m_Netinfo.WORKPATH, pNet->m_Netinfo.WORKPATH.GetLength());
//
//				lock(&temp->m_SocketLock);
//				// 获取挂载点
//				temp->MountPoint = cstrinfo.GetAt(index++);
//				// 获取显示标识
//				temp->sitelable = cstrinfo.GetAt(index++);
//				// 获取所在的子网名称
//				temp->rtcm_fwrt.m_NetName = NetName;
//				// 获取测站类型 1 基准站 2 移动站 3 广播星历
//				cstr = cstrinfo.GetAt(index++);
//				temp->sitetype = atoi(cstr.GetBuffer(0));
//				cstr.ReleaseBuffer();
//				// 获取  连接类型  0=TCP CLIENT  1=TCP SERVER  2=NTRIP CLIENT
//				cstr = cstrinfo.GetAt(index++);
//				temp->m_ConnectType = (ConnectType)(atoi(cstr.GetBuffer(0)));
//				cstr.ReleaseBuffer();
//				// 获取  IP地址
//				temp->IP = cstrinfo.GetAt(index++);
//				// 获取  PORT端口号
//				temp->Port = cstrinfo.GetAt(index++);
//				// 获取  用户名密码
//				temp->User = cstrinfo.GetAt(index++);
//				temp->Pwd = cstrinfo.GetAt(index++);
//				// 获取 测站经纬度
//				cstr = cstrinfo.GetAt(index++);
//				temp->XYZ[0] = (atof(cstr.GetBuffer(0)));
//				cstr.ReleaseBuffer();
//				cstr = cstrinfo.GetAt(index++);
//				temp->XYZ[1] = (atof(cstr.GetBuffer(0)));
//				cstr.ReleaseBuffer();
//				cstr = cstrinfo.GetAt(index++);
//				temp->XYZ[2] = (atof(cstr.GetBuffer(0)));
//				cstr.ReleaseBuffer();
//				ecef2pos(temp->XYZ, temp->BLH);
//				temp->netname = NetName;
//				unlock(&temp->m_SocketLock);
//				NetTemp[temp->rtcm_fwrt.m_NetName]->m_SiteList.insert(make_pair(temp->MountPoint.MakeUpper(), temp));
//			}
//		}
//	}
//	catch (CDBException* e)
//	{
//		cstr.Format("ERROR: void CMainFrame::UpdateNetList(void)   %s ", e->m_strError);
//	}
//	// 遍历原列表，删除列表多余的测站
//	NetList::iterator Net_i(theApp.m_NetList.begin());
//	while (Net_i != theApp.m_NetList.end())
//	{
//		map<CString, CNtripClient*>::iterator site_i(Net_i->second->m_SiteList.begin());
//		while (site_i != Net_i->second->m_SiteList.end())
//		{
//			// 如果找着则把临时中的删除，否则删除原列表中多余的(因为迭代器的原因不能直接删除)
//			if (!(NetTemp.find(Net_i->first) != NetTemp.end() && NetTemp.find(Net_i->first)->second->m_SiteList.find(site_i->first) != NetTemp.find(Net_i->first)->second->m_SiteList.end()))
//			{
//				CNtripClient* pClient = (CNtripClient*)site_i->second;
//				pClient->~CNtripClient();
//				site_i->second = NULL;
//			}
//			site_i++;
//		}
//		Net_i++;
//	}
//	// 删除指针被置空的列表
//	theApp.m_NetList.clear();
//	theApp.m_NetList.insert(NetTemp.begin(), NetTemp.end());
//}
