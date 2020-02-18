#include "stdafx.h"
#include "globalfunc.h"
#include "EBAS_V_UI_R.h"
#include "msql.h"

EBAS_V_UI_RApp theApp;

static void clearnetlist()
{
	for (NetList::iterator i = theApp.m_NetList.begin(); i != theApp.m_NetList.end(); i++)
	{
		for (SiteList::iterator j = i.value()->m_SiteList.begin(); j != i.value()->m_SiteList.end(); j++)
		{
			if (j.value()->psock.state)
			{
				j.value()->close();
			}
			delete j.value();
		}
		i.value()->m_SiteList.clear();
	}
	theApp.m_NetList.clear();
}

void UpdateNetList()
{
	clearnetlist();
	NetList NetTemp;
	QString cstr;
	// �����б�
	//try
	{
		QStringList   cstrList;
		// ��ȡ���������б�
		m_sql.getnetlist(cstrList);
		int size = cstrList.count();
		for (int i = 0; i < size; i++)
		{
			// ��ȡ��������
			QString NetName = cstrList.at(i);
			NetStr* pNet = new NetStr();
			pNet->m_runflag = false;
			NetTemp[NetName] = pNet;
			m_sql.update_netinfo2str(NetName, &pNet->m_Netinfo);
			//if (theApp.m_NetList.find(NetName) != theApp.m_NetList.end())
			//{
			//	pNet->m_Server = theApp.m_NetList.find(NetName)->second->m_Server;
			//}
			//else
			//{
			//	// start server
			//}
			// ���ص��б�ɸѡ
			QStringList MountPointList;
			m_sql.getsitelist(NetName, MountPointList);
			int MPsize = MountPointList.count();
			for (int site_i = 0; site_i < MPsize; site_i++)
			{
				// ��ȡID��
				cstr = MountPointList.at(site_i);
				QStringList cstrinfo;
				m_sql.getsiteinfo(cstr, cstrinfo);
				if (cstrinfo.count())
					cstr = cstrinfo.at(0);
				else
					continue;
				if (theApp.m_NetList.find(NetName) != theApp.m_NetList.end()
					&& theApp.m_NetList.find(NetName).value()->m_SiteList.find(cstr)
					!= theApp.m_NetList.find(NetName).value()->m_SiteList.end())
				{
					NetTemp[NetName]->m_SiteList[cstr] = theApp.m_NetList.find(NetName).value()->m_SiteList.find(cstr).value();
					continue;
				}
				/*"ID","MOUNTPOINT","COM_TYPE","IP","PORT","USERNAME","PASSWORD","LATITUDE","LONGITUDE","HEIGHT"*/
				int index = 1;
				CNtripClient*  temp = new CNtripClient;

				//sprintf(temp->rtcm_fwrt.workpath, (pNet->m_Netinfo.WORKPATH).toUtf8().data(), pNet->m_Netinfo.WORKPATH.length());

				//lock(&temp->m_SocketLock);
				//// ��ȡ���ص�
				//temp->MountPoint = cstrinfo.at(index++);
				//// ��ȡ��ʾ��ʶ
				//temp->sitelable = cstrinfo.at(index++);
				//// ��ȡ���ڵ���������
				//temp->rtcm_fwrt.m_NetName = NetName;
				//// ��ȡ��վ���� 1 ��׼վ 2 �ƶ�վ 3 �㲥����
				//cstr = cstrinfo.GetAt(index++);
				//temp->sitetype = atoi(cstr.GetBuffer(0));
				//cstr.ReleaseBuffer();
				//// ��ȡ  ��������  0=TCP CLIENT  1=TCP SERVER  2=NTRIP CLIENT
				//cstr = cstrinfo.GetAt(index++);
				//temp->m_ConnectType = (ConnectType)(atoi(cstr.GetBuffer(0)));
				//cstr.ReleaseBuffer();
				//// ��ȡ  IP��ַ
				//temp->IP = cstrinfo.GetAt(index++);
				//// ��ȡ  PORT�˿ں�
				//temp->Port = cstrinfo.GetAt(index++);
				//// ��ȡ  �û�������
				//temp->User = cstrinfo.GetAt(index++);
				//temp->Pwd = cstrinfo.GetAt(index++);
				//// ��ȡ ��վ��γ��
				//cstr = cstrinfo.GetAt(index++);
				//temp->XYZ[0] = (atof(cstr.GetBuffer(0)));
				//cstr.ReleaseBuffer();
				//cstr = cstrinfo.GetAt(index++);
				//temp->XYZ[1] = (atof(cstr.GetBuffer(0)));
				//cstr.ReleaseBuffer();
				//cstr = cstrinfo.GetAt(index++);
				//temp->XYZ[2] = (atof(cstr.GetBuffer(0)));
				//cstr.ReleaseBuffer();
				//ecef2pos(temp->XYZ, temp->BLH);
				//temp->netname = NetName;
				//unlock(&temp->m_SocketLock);
				//NetTemp[temp->rtcm_fwrt.m_NetName]->m_SiteList.insert(make_pair(temp->MountPoint.MakeUpper(), temp));
			}
		}
	}
	//catch (CDBException* e)
	//{
	//	cstr.Format("ERROR: void CMainFrame::UpdateNetList(void)   %s ", e->m_strError);
	//}
	// ����ԭ�б�ɾ���б����Ĳ�վ
	//NetList::iterator Net_i(theApp.m_NetList.begin());
	//while (Net_i != theApp.m_NetList.end())
	//{
	//	map<CString, CNtripClient*>::iterator site_i(Net_i->second->m_SiteList.begin());
	//	while (site_i != Net_i->second->m_SiteList.end())
	//	{
	//		// ������������ʱ�е�ɾ��������ɾ��ԭ�б��ж����(��Ϊ��������ԭ����ֱ��ɾ��)
	//		if (!(NetTemp.find(Net_i->first) != NetTemp.end() && NetTemp.find(Net_i->first)->second->m_SiteList.find(site_i->first) != NetTemp.find(Net_i->first)->second->m_SiteList.end()))
	//		{
	//			CNtripClient* pClient = (CNtripClient*)site_i->second;
	//			pClient->~CNtripClient();
	//			site_i->second = NULL;
	//		}
	//		site_i++;
	//	}
	//	Net_i++;
	//}
	//// ɾ��ָ�뱻�ÿյ��б�
	//theApp.m_NetList.clear();
	//theApp.m_NetList.insert(NetTemp.begin(), NetTemp.end());
}

